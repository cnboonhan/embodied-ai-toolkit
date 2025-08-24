# embodied-ai-toolkit

Tools to use for Humanoid Data Collection.

## Setup
```
apt update
uv venv -p 3.11
source .venv/bin/activate
uv sync
# Install grpcurl: https://github.com/fullstorydev/grpcurl/releases
# Install Rust ( only for building ): https://www.rust-lang.org/tools/install
apt install -y llvm-dev libclang-dev clang libopencv-dev  gcc-aarch64-linux-gnu gcc-arm-linux-gnueabihf

cd streamer
cargo build  
cargo build --release --target x86_64-unknown-linux-gnu 
CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc cargo build --release --target aarch64-unknown-linux-gnu
find . -name "streamer"

uv run main.py --config_path schema/config.json
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


