# embodied-ai-toolkit

Toolkit for robot control and data collection. Focus is on humanoid robots.

## Installation

```bash
# Clone and setup
git clone --recurse-submodules https://github.com/cnboonhan/embodied-ai-toolkit
cd embodied-ai-toolkit

# Install dependencies with uv
uv venv -p 3.11
source .venv/bin/activate
uv sync
uv pip install -e pyroki
```

## Parameters

| Parameter | Type | Purpose |
|-----------|------|---------|
| `path` | str | URDF file path, directory, or robot description name |
| `vis_port` | int | Visualization server port |
| `api_port` | int | API server port |
| `ik_targets` | str | Comma-separated IK target links |
| `custom_joints` | str | Custom joints (name:lower:upper) |
| `joint_limits` | str | Override joint limits (name:lower:upper) |
| `exponential_alpha` | float | Smoothing factor (0.01-1.0) |
| `history_length` | int | Smoothing history length |
| `change_tolerance` | float | Minimum change threshold |
| `label` | str | Window title/label for the visualization |

## Launch Commands

```bash
# Basic launch with path to custom urdf
python src/robot_server.py --path path/to/robot.urdf --vis-port 8080 --api-port 5000

# Launch with robot description
python src/robot_server.py --path g1_description --vis-port 8080 --api-port 5000

# With IK targets. This enables IK movement of the specified links
python src/robot_server.py --path g1_description --vis-port 8080 --api-port 5000 --ik-targets "left_palm_link,right_palm_link"

# With custom joints added. Schema is joint:lower_limit:upper_limit
python src/robot_server.py --path g1_description --vis-port 8080 --api-port 5000 --custom-joints "left_hand:0.0:1.0,right_hand:0.0:1.0"

# With custom window label
python src/robot_server.py --path g1_description --vis-port 8080 --api-port 5000 --label "My Robot"

# Full example with custom parameters and smoothing
python src/robot_server.py --path g1_description \
  --vis-port 8080 \
  --api-port 5000 \
  --ik-targets "left_palm_link,right_palm_link,torso_link" \
  --custom-joints "left_hand:0.0:1.0,right_hand:0.0:1.0" \
  --joint-limits "left_hip_pitch_joint:-1.0:1.0,torso_joint:-0.5:0.5" \
  --exponential-alpha 0.05 \
  --history-length 15 \
  --change-tolerance 0.01 \
  --label "G1 Robot Controller"
```

## API Endpoints

- `GET /get_joints` - Get all joint information
- `POST /update_joints` - Update joint values (direct format: `{"joint_name": {"value": 1.0}}`)

```bash
# get joints
curl -X GET http://localhost:5000/get_joints

# Update joints. This can be streamed from a teleoperation device
curl -X POST http://localhost:5000/update_joints -H "Content-Type: application/json" -d '{"left_hand": {"value": 0.0}, "right_hand": {"value": 1.2}}'
```

## Data Collection Minimal Example
```
# Data Visualizer
rerun --serve-web --web-viewer-port 9090 --port 9876 
# Robot Action Controller
python src/robot_server.py --label "G1 Action Controller" --path g1_description --vis-port 8080 --api-port 5000 --ik_targets "left_palm_link,right_palm_link,torso_link" --custom-joints "left_hand:0.0:1.0,right_hand:0.0:1.0"
# Robot State Visualizer
python src/robot_server.py --label "G1 State Visualizer" --path g1_description --vis-port 8081 --api-port 5001 --custom-joints "left_hand:0.0:1.0,right_hand:0.0:1.0"
# Mock Control Loop
while true; do curl -s -X GET "http://127.0.0.1:5000/get_joints" | curl -s -X POST "http://127.0.0.1:5001/update_joints" -H "Content-Type: application/json" -d @-; sleep 0.05; done
```
