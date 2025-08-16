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

| Parameter | Type | Default | Purpose |
|-----------|------|---------|---------|
| `path` | str | - | URDF file path, directory, or robot description name |
| `vis_port` | int | - | Visualization server port |
| `api_port` | int | - | API server port |
| `ik_targets` | str | None | Comma-separated IK target links |
| `custom_joints` | str | None | Custom joints (name:lower:upper) |
| `joint_limits` | str | None | Override joint limits (name:lower:upper) |
| `exponential_alpha` | float | 0.05 | Smoothing factor (0.01-1.0) |
| `history_length` | int | 15 | Smoothing history length |
| `change_tolerance` | float | 0.01 | Minimum change threshold |

## Launch Commands

```bash
# Basic launch
python src/main.py --path path/to/robot.urdf --vis-port 8080 --api-port 5000

# Launch with robot description
python src/main.py --path g1_description --vis-port 8080 --api-port 5000

# With IK targets
python src/main.py --path g1_description --vis-port 8080 --api-port 5000 --ik-targets "left_palm_link,right_palm_link"

# With custom joints
python src/main.py --path g1_description --vis-port 8080 --api-port 5000 --custom-joints "left_hand:0.0:1.0,right_hand:0.0:1.0"

# Full example with custom parameters and smoothing
python src/main.py --path g1_description \
  --vis-port 8080 \
  --api-port 5000 \
  --ik-targets "left_palm_link,right_palm_link,torso_link" \
  --custom-joints "left_hand:0.0:1.0,right_hand:0.0:1.0" \
  --joint-limits "left_hip_pitch_joint:-1.0:1.0,torso_joint:-0.5:0.5" \
  --exponential-alpha 0.05 \
  --history-length 15 \
  --change-tolerance 0.01
```

## API Endpoints

- `GET /get_joints` - Get all joint information
- `POST /update_joints` - Update joint values

```bash
# get joints
curl -X GET http://localhost:5000/get_joints

# Update joints
curl -X POST http://localhost:5000/update_joints -H "Content-Type: application/json" -d '{"joints": {"left_hand": {"value": 0.0}, "right_hand": {"value": 1.2}}}'
```
