from __future__ import annotations
from flask import Flask, request, jsonify
from utils import RobotConfig


class FlaskAPIServer:

    def __init__(self, robot_config: RobotConfig, port: int):
        self.robot_config = robot_config
        self.app = Flask(__name__)
        self.port = port
        self._setup_routes()

    def _setup_routes(self):
        @self.app.route("/update_joints", methods=["POST"])
        def update_joints():
            try:
                joints_data = request.get_json()
                if not joints_data:
                    return jsonify({"error": "No JSON data provided"}), 400

                if not isinstance(joints_data, dict):
                    return (
                        jsonify(
                            {
                                "error": "joints must be a dictionary mapping joint names to their data"
                            }
                        ),
                        400,
                    )

                # Extract joint values from the data structure
                joint_values = {}
                custom_values = {}
                
                for joint_name, joint_data in joints_data.items():
                    if isinstance(joint_data, dict) and "value" in joint_data:
                        value = joint_data["value"]
                        
                        # Check if it's a URDF joint or custom joint
                        if joint_name in self.robot_config.get_sliders(is_custom=False):
                            joint_values[joint_name] = value
                        elif joint_name in self.robot_config.get_sliders(is_custom=True):
                            custom_values[joint_name] = value
                        else:
                            return jsonify({"error": f"Joint '{joint_name}' not found"}), 400
                    else:
                        return jsonify({"error": f"Invalid data format for joint {joint_name}"}), 400
                
                # Update both URDF joints and custom joints
                joint_result = self.robot_config.update_multiple_values(joint_values, is_custom=False) if joint_values else {"success": True, "results": {}}
                custom_result = self.robot_config.update_multiple_values(custom_values, is_custom=True) if custom_values else {"success": True, "results": {}}
                
                # Combine results
                combined_result = {
                    "success": joint_result.get("success", True) and custom_result.get("success", True),
                    "updated_joints": joint_result.get("updated_joints", []),
                    "updated_custom_joints": custom_result.get("updated_custom_joints", [])
                }
                
                return jsonify(combined_result)

            except ValueError as e:
                return jsonify({"error": str(e)}), 400
            except Exception as e:
                return jsonify({"error": str(e)}), 500

        @self.app.route("/get_joints", methods=["GET"])
        def get_joints():
            try:
                # Get both URDF joints and custom joints
                joints_info = self.robot_config.get_info(is_custom=False)
                custom_joints_info = self.robot_config.get_info(is_custom=True)
                
                # Combine both into a single response
                all_joints = {**joints_info, **custom_joints_info}
                return jsonify(all_joints)
            except Exception as e:
                return jsonify({"error": str(e)}), 500

    def run(self):
        """Start the Flask server."""
        self.app.run(host="0.0.0.0", port=self.port, debug=False, use_reloader=False)
