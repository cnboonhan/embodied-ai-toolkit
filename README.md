# embodied-ai-toolkit

Tools to use for Humanoid Data Collection.

## Setup
```
apt update
uv venv -p 3.12
source .venv/bin/activate
uv sync
# Install grpcurl: https://github.com/fullstorydev/grpcurl/releases

uv run main.py --config_path schema/config.example.json
uv run joint_streamer.py --config-path ./schema/config.example.json --update-frequency 10
```

## GRPC API Calls
```
grpcurl -plaintext localhost:5000 list
grpcurl -plaintext localhost:5000 list rosbot_api.RobotApiService

# Get Robot Info
grpcurl -plaintext -format json localhost:5000 rosbot_api.RobotApiService/GetRobotInfo

# Update Joints
grpcurl -plaintext -d '{
  "joint_updates": {
    "joint_1": 1.57,
    "joint_2": 0.785,
    "custom_joint1": 0.5
  }
}' localhost:5000 rosbot_api.RobotApiService/UpdateJoints

# Stream Joints
grpcurl -format json -plaintext -d '{
  "update_frequency": 100.0
}' localhost:5000 rosbot_api.RobotApiService/StreamJointData
```


