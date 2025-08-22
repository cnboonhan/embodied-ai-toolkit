from robot_descriptions.loaders.yourdfpy import load_robot_description
from pathlib import Path
import yourdfpy
import asyncio
import multiprocessing
import numpy as np
import viser
from viser.extras import ViserUrdf
from dataclasses import dataclass
from typing import List, Optional, Tuple
import asyncio
import json
import threading
from aiohttp import web, WSMsgType

@dataclass
class CustomJoint:
    name: str
    type: str
    limits: Tuple[float, float]

@dataclass
class Config:
    urdf_path: str
    api_port: int
    visualization_port: int
    custom_joints: Optional[List[CustomJoint]] = None

def load_config(config_path: Path) -> Config:
    import json
    
    with open(config_path, 'r') as f:
        data = json.load(f)
    
    # Convert custom_joints if present
    custom_joints = None
    if 'custom_joints' in data:
        custom_joints = [
            CustomJoint(
                name=joint['name'],
                type=joint['type'],
                limits=tuple(joint['limits'])
            )
            for joint in data['custom_joints']
        ]
    
    return Config(
        urdf_path=data['urdf_path'],
        api_port=data['api_port'],
        visualization_port=data['visualization_port'],
        custom_joints=custom_joints
    )

def load_urdf(urdf_path: str):
    try:
        if Path(urdf_path).exists():
            return yourdfpy.URDF.load(urdf_path)
        else: 
            return load_robot_description(urdf_path)
    except Exception as e:
        raise Exception(f"Failed to load URDF from local or robot_descriptions '{urdf_path}'")

def setup_ui(server: viser.ViserServer, robot: ViserUrdf, config: Config):
    slider_names: list[str] = []
    slider_handles: list[viser.GuiInputHandle[float]] = []
    custom_slider_handles: list[viser.GuiInputHandle[float]] = []
    custom_slider_names: list[str] = []

    with server.gui.add_folder("Joint position controls"):
        for joint_name, (lower,upper,) in robot.get_actuated_joint_limits().items():
            lower = lower if lower is not None else -np.pi
            upper = upper if upper is not None else np.pi
            initial_pos = (lower + upper) / 2.0

            slider = server.gui.add_slider(
                label=joint_name,
                min=lower,
                max=upper,
                step=1e-3,
                initial_value=initial_pos,
            )
            slider.on_update(lambda _, s=slider: robot.update_cfg(np.array([slider.value for slider in slider_handles])))
            slider_handles.append(slider)
            slider_names.append(joint_name)

    if config.custom_joints:
        with server.gui.add_folder("Custom position controls"):
            for custom_joint in config.custom_joints:
                lower, upper = custom_joint.limits
                initial_pos = (lower + upper) / 2.0

                slider = server.gui.add_slider(
                    label=custom_joint.name,
                    min=lower,
                    max=upper,
                    step=1e-3,
                    initial_value=initial_pos,
                )
                # slider.on_update(lambda _, s=slider: robot.update_cfg(np.array(np.array([slider.value for slider in slider_handles]))))
                custom_slider_handles.append(slider)
                custom_slider_names.append(custom_joint.name)

    robot.update_cfg(np.array([slider.value for slider in slider_handles]))
    return slider_handles, slider_names, custom_slider_handles, custom_slider_names

class WebSocketServer:
    def __init__(self, slider_handles: List[viser.GuiInputHandle[float]], slider_names: List[str], 
                 custom_slider_handles: List[viser.GuiInputHandle[float]], custom_slider_names: List[str], 
                 robot: ViserUrdf, port: int = 50051):
        self.slider_handles = slider_handles
        self.slider_names = slider_names
        self.custom_slider_handles = custom_slider_handles
        self.custom_slider_names = custom_slider_names
        self.robot = robot
        self.port = port
        self.clients = set()
        self.app = web.Application()
        self.app.router.add_get('/ws', self.websocket_handler)
        
    async def websocket_handler(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        
        self.clients.add(ws)
        print(f"WebSocket client connected. Total clients: {len(self.clients)}")
        
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    if msg.data == 'get_joints':
                        joints = {}
                        for i, joint_name in enumerate(self.slider_names):
                            joints[joint_name] = self.slider_handles[i].value
                        for i, joint_name in enumerate(self.custom_slider_names):
                            joints[joint_name] = self.custom_slider_handles[i].value
                        await ws.send_json(joints)
                        
                    elif msg.data == 'get_limits':
                        limits = {}
                        for i, slider in enumerate(self.slider_handles):
                            limits[self.slider_names[i]] = {
                                'lower': slider.min,
                                'upper': slider.max
                            }
                        for i, slider in enumerate(self.custom_slider_handles):
                            limits[self.custom_slider_names[i]] = {
                                'lower': slider.min,
                                'upper': slider.max
                            }
                        await ws.send_json(limits)
                        
                    elif msg.data.startswith('/update_joint'):
                        try:
                            json_str = msg.data[len('/update_joint'):].strip()
                            if json_str:
                                updates = json.loads(json_str)
                                
                                invalid_joints = []
                                for joint_name in updates.keys():
                                    if joint_name not in self.slider_names and joint_name not in self.custom_slider_names:
                                        invalid_joints.append(joint_name)
                                
                                if invalid_joints:
                                    raise Exception(f"Invalid joints: {invalid_joints}")
                                
                                updated_joints = []
                                for joint_name, angle in updates.items():
                                    if joint_name in self.slider_names:
                                        joint_index = self.slider_names.index(joint_name)
                                        self.slider_handles[joint_index].value = angle
                                        updated_joints.append(joint_name)
                                    
                                    elif joint_name in self.custom_slider_names:
                                        joint_index = self.custom_slider_names.index(joint_name)
                                        self.custom_slider_handles[joint_index].value = angle
                                        updated_joints.append(joint_name)
                                
                                await ws.send_json({
                                    'success': True,
                                    'updated_joints': updated_joints
                                })
                            else:
                                await ws.send_json({
                                    'success': False,
                                    'error': 'No joint data provided'
                                })
                        except json.JSONDecodeError:
                            await ws.send_json({
                                'success': False,
                                'error': 'Invalid JSON format'
                            })
                        except Exception as e:
                            await ws.send_json({
                                'success': False,
                                'error': str(e)
                            })

                elif msg.type == WSMsgType.ERROR:
                    print(f'WebSocket error: {ws.exception()}')
        except Exception as e:
            print(f"WebSocket error: {e}")
        finally:
            self.clients.discard(ws)
            print(f"WebSocket client disconnected. Total clients: {len(self.clients)}")
        
        return ws
    
    def start(self):
        def run_server():
            
            async def start_server():
                runner = web.AppRunner(self.app)
                await runner.setup()
                site = web.TCPSite(runner, 'localhost', self.port)
                await site.start()
                print(f"WebSocket server started on port {self.port}")
                print(f"WebSocket endpoint: ws://localhost:{self.port}/ws")
                
                while True:
                    await asyncio.sleep(1)
            
            asyncio.run(start_server())
        
        self.thread = threading.Thread(target=run_server, daemon=True)
        self.thread.start()
            
def start_websocket_server(slider_handles: List[viser.GuiInputHandle[float]], slider_names: List[str],
                          custom_slider_handles: List[viser.GuiInputHandle[float]], custom_slider_names: List[str],
                          robot: ViserUrdf, port: int = 50051):
    server = WebSocketServer(slider_handles, slider_names, custom_slider_handles, custom_slider_names, robot, port)
    server.start()
    return server