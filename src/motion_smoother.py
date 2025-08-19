#!/usr/bin/env python3

# websocat --no-close ws://localhost:5001/ws/smoothed_joints 
"""
Motion Smoother
Subscribes to websocket at 127.0.0.1:5000/ws/update_joints, applies CCMA smoothing,
and broadcasts smoothed trajectory to another websocket endpoint.
"""

import asyncio
import json
import time
import argparse
from collections import defaultdict, deque
import numpy as np
import websockets
import logging
import aiohttp
from aiohttp import web, WSMsgType

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='Motion smoother with websocket subscription and broadcasting')
    parser.add_argument(
        '--input-ws-url',
        type=str,
        default='ws://127.0.0.1:5000/ws/update_joints',
        help='Input websocket URL (default: ws://127.0.0.1:5000/ws/update_joints)'
    )
    parser.add_argument(
        '--output-ws-url',
        type=str,
        default='ws://127.0.0.1:5001/ws/smoothed_joints',
        help='Output websocket URL (default: ws://127.0.0.1:5001/ws/smoothed_joints)'
    )
    parser.add_argument(
        '--smoothing-alpha',
        type=float,
        default=0.15,
        help='Exponential smoothing alpha (0.1-0.3, lower=smother, default: 0.15)'
    )
    parser.add_argument(
        '--max-velocity-rad-s',
        type=float,
        default=0.3,
        help='Maximum joint velocity in rad/s (default: 0.3)'
    )
    parser.add_argument(
        '--max-acceleration-rad-s2',
        type=float,
        default=1.0,
        help='Maximum joint acceleration in rad/s² (default: 1.0)'
    )
    parser.add_argument(
        '--output-rate-hz',
        type=float,
        default=30.0,
        help='Output rate in Hz (default: 30.0)'
    )
    return parser.parse_args()

def generate_smoothed_values(joint_histories, smoothing_params, previous_positions=None, previous_velocities=None, dt=0.033):
    """Generate smoothed values using exponential smoothing with velocity and acceleration limiting"""
    smoothed_values = {}
    
    alpha = smoothing_params['alpha']
    max_velocity = smoothing_params['max_velocity']
    max_acceleration = smoothing_params['max_acceleration']
    
    for joint_name, history in joint_histories.items():
        if len(history) > 0:
            current_raw_value = history[-1]
            
            # Get previous smoothed position and velocity
            prev_position = previous_positions.get(joint_name, current_raw_value)
            prev_velocity = previous_velocities.get(joint_name, 0.0)
            
            # Apply exponential smoothing
            smoothed_position = alpha * current_raw_value + (1 - alpha) * prev_position
            
            # Calculate current velocity
            current_velocity = (smoothed_position - prev_position) / dt
            
            # Limit velocity
            if abs(current_velocity) > max_velocity:
                if current_velocity > 0:
                    current_velocity = max_velocity
                else:
                    current_velocity = -max_velocity
                # Recalculate position based on limited velocity
                smoothed_position = prev_position + current_velocity * dt
            
            # Calculate acceleration
            current_acceleration = (current_velocity - prev_velocity) / dt
            
            # Limit acceleration
            if abs(current_acceleration) > max_acceleration:
                if current_acceleration > 0:
                    current_acceleration = max_acceleration
                else:
                    current_acceleration = -max_acceleration
                # Recalculate velocity and position based on limited acceleration
                current_velocity = prev_velocity + current_acceleration * dt
                smoothed_position = prev_position + current_velocity * dt
            
            smoothed_values[joint_name] = smoothed_position
            
            # Update previous values for next iteration
            previous_positions[joint_name] = smoothed_position
            previous_velocities[joint_name] = current_velocity
            
        else:
            smoothed_values[joint_name] = None
    
    return smoothed_values

class MotionSmoother:
    def __init__(self, args):
        self.args = args
        self.input_ws_url = args.input_ws_url
        self.output_ws_url = args.output_ws_url
        
        # Smoothing parameters
        self.smoothing_params = {
            'alpha': args.smoothing_alpha,
            'max_velocity': args.max_velocity_rad_s,
            'max_acceleration': args.max_acceleration_rad_s2
        }
        
        # Joint history tracking (minimal history for exponential smoothing)
        self.joint_history = defaultdict(lambda: deque(maxlen=2))  # Only need last few values
        self.previous_positions = {}
        self.previous_velocities = {}
        
        # Rate limiting
        self.output_rate = args.output_rate_hz
        self.output_interval = 1.0 / self.output_rate
        self.last_output_time = 0.0
        self.pending_smoothed_values = None
        
        # Performance monitoring
        self.processing_times = deque(maxlen=100)
        self.last_performance_log = time.time()
        self.output_count = 0
        
        # Websocket clients
        self.input_ws = None
        self.output_ws = None
        self.output_clients = set()
        
        logger.info(f"Motion Smoother initialized:")
        logger.info(f"  Input WS: {self.input_ws_url}")
        logger.info(f"  Output WS: {self.output_ws_url}")
        logger.info(f"  Smoothing alpha: {self.smoothing_params['alpha']}")
        logger.info(f"  Max velocity: {self.smoothing_params['max_velocity']} rad/s")
        logger.info(f"  Max acceleration: {self.smoothing_params['max_acceleration']} rad/s²")
        logger.info(f"  Output rate: {self.output_rate} Hz")

    def add_to_joint_history(self, joint_name, value):
        """Add a value to the joint history"""
        self.joint_history[joint_name].append(value)

    async def start_output_server(self):
        """Start the output websocket server using aiohttp"""
        app = web.Application()
        
        async def websocket_handler(request):
            ws = web.WebSocketResponse()
            await ws.prepare(request)
            
            self.output_clients.add(ws)
            logger.info(f"Output client connected. Total clients: {len(self.output_clients)}")
            
            try:
                # Send initial connection message
                await ws.send_json({"status": "connected", "message": "Motion smoother ready"})
                
                async for msg in ws:
                    if msg.type == WSMsgType.TEXT:
                        try:
                            data = json.loads(msg.data)
                            logger.debug(f"Received message from client: {data}")
                        except json.JSONDecodeError:
                            logger.warning(f"Received non-JSON message: {msg.data}")
                    elif msg.type == WSMsgType.ERROR:
                        logger.error(f"WebSocket error: {ws.exception()}")
                    elif msg.type == WSMsgType.CLOSE:
                        break
                        
            except Exception as e:
                logger.error(f"Error handling client connection: {e}")
            finally:
                self.output_clients.discard(ws)
                logger.info(f"Output client disconnected. Total clients: {len(self.output_clients)}")
            
            return ws
        
        # Add websocket route
        app.router.add_get('/ws/smoothed_joints', websocket_handler)
        
        # Add a simple health check endpoint
        async def health_check(request):
            return web.json_response({
                "status": "healthy",
                "clients": len(self.output_clients),
                "uptime": time.time() - self.start_time if hasattr(self, 'start_time') else 0
            })
        
        app.router.add_get('/health', health_check)
        
        # Parse URL to get host and port
        if self.output_ws_url.startswith('ws://'):
            url = self.output_ws_url[5:]  # Remove 'ws://'
        else:
            url = self.output_ws_url
        
        if ':' in url:
            host, port_str = url.split(':', 1)
            if '/' in port_str:
                port_str = port_str.split('/', 1)[0]
            port = int(port_str)
        else:
            host = url
            port = 5001
        
        logger.info(f"Starting output server on {host}:{port}")
        
        # Store the runner for cleanup
        self.web_runner = web.AppRunner(app)
        await self.web_runner.setup()
        self.web_site = web.TCPSite(self.web_runner, host, port)
        await self.web_site.start()
        
        return self.web_runner

    async def connect_input_websocket(self):
        """Connect to the input websocket"""
        while True:
            try:
                logger.info(f"Connecting to input websocket: {self.input_ws_url}")
                self.input_ws = await websockets.connect(
                    self.input_ws_url,
                    ping_interval=None,
                    close_timeout=10
                )
                logger.info("Connected to input websocket successfully!")
                return
            except Exception as e:
                logger.error(f"Failed to connect to input websocket: {e}")
                await asyncio.sleep(2)

    async def process_input_message(self, message):
        """Process incoming websocket message"""
        try:
            start_time = time.time()
            
            # Parse the message
            if isinstance(message, str):
                data = json.loads(message)
            else:
                data = message
            
            logger.debug(f"Received input message: {data}")
            
            # Extract joint data
            for joint_name, joint_data in data.items():
                if isinstance(joint_data, dict) and 'value' in joint_data:
                    self.add_to_joint_history(joint_name, joint_data['value'])
            
            # Check if we have enough history for smoothing (minimal for exponential smoothing)
            min_points_needed = 2  # Just need current and previous value
            all_joints_ready = all(len(history) >= min_points_needed for history in self.joint_history.values() if len(history) > 0)
            
            if all_joints_ready:
                # Generate smoothed values
                smoothed_values = generate_smoothed_values(
                    self.joint_history, 
                    self.smoothing_params, 
                    self.previous_positions, 
                    self.previous_velocities,
                    dt=self.output_interval
                )
                
                # Store for rate-limited output
                self.pending_smoothed_values = smoothed_values
                logger.debug(f"Generated smoothed values: {smoothed_values}")
            else:
                logger.debug("Waiting for sufficient joint history...")
            
            # Record processing time
            processing_time = time.time() - start_time
            self.processing_times.append(processing_time)
            
            # Log performance periodically
            current_time = time.time()
            if current_time - self.last_performance_log > 10.0:
                avg_processing_time = sum(self.processing_times) / len(self.processing_times) if self.processing_times else 0
                max_processing_time = max(self.processing_times) if self.processing_times else 0
                logger.info(f"📊 Performance: avg={avg_processing_time*1000:.1f}ms, max={max_processing_time*1000:.1f}ms, samples={len(self.processing_times)}")
                self.last_performance_log = current_time
                
        except Exception as e:
            logger.error(f"Error processing input message: {e}")

    async def broadcast_smoothed_values(self):
        """Broadcast smoothed values to output clients"""
        if not self.pending_smoothed_values or not self.output_clients:
            return
        
        current_time = time.time()
        if current_time - self.last_output_time >= self.output_interval:
            try:
                # Prepare message in the same format as input
                output_message = {}
                for joint_name, value in self.pending_smoothed_values.items():
                    if value is not None:
                        output_message[joint_name] = {"value": value}
                
                if output_message:
                    # Broadcast to all connected clients
                    disconnected_clients = set()
                    for client in self.output_clients:
                        try:
                            # Check if connection is still open
                            if not client.closed:
                                await client.send_json(output_message)
                            else:
                                disconnected_clients.add(client)
                        except Exception as e:
                            logger.warning(f"Error sending to client: {e}")
                            disconnected_clients.add(client)
                    
                    # Remove disconnected clients
                    self.output_clients -= disconnected_clients
                    
                    if disconnected_clients:
                        logger.info(f"Removed {len(disconnected_clients)} disconnected clients")
                    
                    self.output_count += 1
                    if self.output_count % 50 == 0:
                        logger.info(f"📤 Broadcast {self.output_count} messages to {len(self.output_clients)} clients")
                else:
                    # Send heartbeat if no joint data
                    heartbeat_message = {"status": "heartbeat", "timestamp": current_time}
                    for client in list(self.output_clients):
                        try:
                            if not client.closed:
                                await client.send_json(heartbeat_message)
                        except:
                            pass
                
                self.last_output_time = current_time
                
            except Exception as e:
                logger.error(f"Error broadcasting smoothed values: {e}")

    async def run(self):
        """Main run loop"""
        # Store start time for health check
        self.start_time = time.time()
        
        # Start output server
        output_server = await self.start_output_server()
        
        # Connect to input websocket
        await self.connect_input_websocket()
        
        try:
            # Main processing loop
            while True:
                # Process input messages
                try:
                    message = await asyncio.wait_for(self.input_ws.recv(), timeout=0.1)
                    await self.process_input_message(message)
                except asyncio.TimeoutError:
                    pass  # No message received, continue
                except websockets.exceptions.ConnectionClosed:
                    logger.warning("Input websocket connection closed, reconnecting...")
                    await self.connect_input_websocket()
                    continue
                except Exception as e:
                    logger.error(f"Error receiving input message: {e}")
                
                # Broadcast smoothed values
                await self.broadcast_smoothed_values()
                
                # Small delay to prevent busy waiting
                await asyncio.sleep(0.001)
                
        except KeyboardInterrupt:
            logger.info("Shutting down motion smoother...")
        finally:
            if self.input_ws:
                await self.input_ws.close()
            if hasattr(self, 'web_runner'):
                await self.web_runner.cleanup()

async def main():
    """Main function"""
    args = parse_arguments()
    smoother = MotionSmoother(args)
    await smoother.run()

if __name__ == "__main__":
    asyncio.run(main())
