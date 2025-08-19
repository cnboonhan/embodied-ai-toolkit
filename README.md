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

# Download and install websocat: https://github.com/vi/websocat/releases
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

# Full example with custom parameters
python src/robot_server.py --path g1_description \
  --vis-port 8080 \
  --api-port 5000 \
  --ik-targets "left_palm_link,right_palm_link,torso_link" \
  --custom-joints "left_hand:0.0:1.0,right_hand:0.0:1.0" \
  --joint-limits "left_hip_pitch_joint:-1.0:1.0,torso_joint:-0.5:0.5" \
  --change-tolerance 0.01 \
  --label "G1 Robot Controller"
```

#### Get Current Joint State
```bash
echo "{}" | websocat ws://localhost:5000/ws/update_joints
```

#### Update Joint Values 
```bash
# Both will set to 0.5.  When limits are specified, values are normalized from provided limits to urdf limits
echo '{"left_hand": {"value": 0.5}, "right_hand": {"value": 0.0, "limits": [-100, 100]}}' | websocat ws://localhost:5000/ws/update_joints
```

#### Real-time Monitoring
```bash
websocat --no-close ws://localhost:5000/ws/update_joints
```
