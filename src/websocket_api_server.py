from __future__ import annotations
import asyncio
import json
import logging
from typing import Dict, Any
from aiohttp import web, WSMsgType
from utils import RobotConfig

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class WebSocketAPIServer:
    """WebSocket API server with separate endpoints for different actions."""

    def __init__(self, robot_config: RobotConfig, port: int, host: str = "0.0.0.0"):
        self.robot_config = robot_config
        self.port = port
        self.host = host
        self.app = web.Application()
        self.websockets = {
            "update_joints": set(),
        }
        self._setup_routes()

    def _setup_routes(self):
        # WebSocket endpoints
        self.app.router.add_get("/ws/update_joints", self.joints_handler)

    async def joints_handler(self, request):
        """Handle joint update WebSocket connections."""
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        self.websockets["update_joints"].add(ws)
        logger.info(
            f"Update Joints WebSocket connected. Total: {len(self.websockets['update_joints'])}"
        )

        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    await self.handle_joints_message(ws, msg.data)
                elif msg.type == WSMsgType.PING:
                    await ws.pong()
                elif msg.type == WSMsgType.CLOSE:
                    break
        except Exception as e:
            logger.error(f"Joints WebSocket error: {e}")
        finally:
            self.websockets["update_joints"].discard(ws)
            logger.info(
                f"Update Joints WebSocket disconnected. Total: {len(self.websockets['update_joints'])}"
            )

        return ws

    async def handle_joints_message(self, ws, message: str):
        """Handle joint update messages."""
        try:
            data = json.loads(message)
            response = await self.update_joints(data)
            await ws.send_str(json.dumps(response))
        except json.JSONDecodeError:
            await ws.send_str(json.dumps({"error": "Invalid JSON"}))
        except Exception as e:
            await ws.send_str(json.dumps({"error": str(e)}))

    async def update_joints(self, joints_data: Dict[str, Any]) -> Dict[str, Any]:
        """Update joint values and return full joint state."""
        try:
            if not isinstance(joints_data, dict):
                return {"error": "Expected joint data dictionary"}

            joint_values = {}
            custom_values = {}

            for joint_name, joint_data_item in joints_data.items():
                if isinstance(joint_data_item, dict) and "value" in joint_data_item:
                    value = joint_data_item["value"]

                    # Check if limits are provided in the joint data
                    if "limits" in joint_data_item and isinstance(joint_data_item["limits"], list) and len(joint_data_item["limits"]) == 2:
                        limits = joint_data_item["limits"]
                        lower_limit, upper_limit = limits[0], limits[1]
                        
                        # Validate limits
                        if not isinstance(lower_limit, (int, float)) or not isinstance(upper_limit, (int, float)):
                            return {"error": f"Invalid limit types for joint {joint_name}"}
                        
                        if lower_limit >= upper_limit:
                            return {"error": f"Invalid limits for joint {joint_name}: lower limit must be less than upper limit"}
                        
                        # Transform the provided value to fit within the provided limits
                        if joint_name in self.robot_config.get_sliders(is_custom=False):
                            transformed_value = self._transform_value_to_limits(value, lower_limit, upper_limit, joint_name, is_custom=False)
                            joint_values[joint_name] = transformed_value
                        elif joint_name in self.robot_config.get_sliders(is_custom=True):
                            transformed_value = self._transform_value_to_limits(value, lower_limit, upper_limit, joint_name, is_custom=True)
                            custom_values[joint_name] = transformed_value
                        else:
                            return {"error": f"Joint '{joint_name}' not found"}
                    else:
                        # No limits provided, use existing limits
                        if joint_name in self.robot_config.get_sliders(is_custom=False):
                            joint_values[joint_name] = value
                        elif joint_name in self.robot_config.get_sliders(is_custom=True):
                            custom_values[joint_name] = value
                        else:
                            return {"error": f"Joint '{joint_name}' not found"}
                else:
                    return {"error": f"Invalid data format for joint {joint_name}"}

            if joint_values:
                self.robot_config.update_multiple_values(joint_values, is_custom=False)
            if custom_values:
                self.robot_config.update_multiple_values(custom_values, is_custom=True)

            joints_info = self.robot_config.get_info(is_custom=False)
            custom_joints_info = self.robot_config.get_info(is_custom=True)
            all_joints = {**joints_info, **custom_joints_info}

            await self.broadcast_update(all_joints)

            return all_joints

        except Exception as e:
            return {"error": str(e)}

    def _transform_value_to_limits(self, provided_value: float, input_lower_limit: float, input_upper_limit: float, joint_name: str, is_custom: bool = False) -> float:
        sliders = self.robot_config.custom_sliders if is_custom else self.robot_config.joint_sliders
        
        if joint_name not in sliders:
            return provided_value
            
        slider = sliders[joint_name]
        slider_min, slider_max = slider.min, slider.max
        
        if input_upper_limit != input_lower_limit:
            normalized_position = (provided_value - input_lower_limit) / (input_upper_limit - input_lower_limit)
            transformed_value = slider_min + normalized_position * (slider_max - slider_min)
            return transformed_value
        else:
            return (slider_min + slider_max) / 2

    async def broadcast_update(self, update_data: Dict[str, Any]):
        """Broadcast updates to all connected clients."""
        if not any(self.websockets.values()):
            return

        message = json.dumps({"type": "joint_update", "data": update_data})

        for endpoint, clients in self.websockets.items():
            disconnected = set()
            for ws in clients:
                try:
                    await ws.send_str(message)
                except Exception as e:
                    logger.error(f"Failed to send to {endpoint} client: {e}")
                    disconnected.add(ws)

            self.websockets[endpoint] -= disconnected

    def run(self):
        """Start the server."""
        logger.info(f"Starting WebSocket API server on {self.host}:{self.port}")
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._run_server())
        finally:
            loop.close()

    async def _run_server(self):
        """Run the server asynchronously."""
        runner = web.AppRunner(self.app)
        await runner.setup()
        site = web.TCPSite(runner, self.host, self.port)
        await site.start()
        logger.info(f"WebSocket API server started on {self.host}:{self.port}")
        
        # Keep the server running
        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            await runner.cleanup()
            logger.info("WebSocket API server stopped")

    async def start_async(self):
        """Start the server asynchronously."""
        logger.info(f"Starting WebSocket API server on {self.host}:{self.port}")
        runner = web.AppRunner(self.app)
        await runner.setup()
        site = web.TCPSite(runner, self.host, self.port)
        await site.start()
        logger.info(f"WebSocket API server started on {self.host}:{self.port}")
        return runner, site

    async def stop_async(self, runner):
        """Stop the server."""
        await runner.cleanup()
        logger.info("WebSocket API server stopped")
