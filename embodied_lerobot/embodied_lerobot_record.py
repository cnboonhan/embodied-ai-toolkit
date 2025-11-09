from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
from embodied_lerobot.robots.urdf_follower import UrdfFollower, UrdfFollowerConfig
from embodied_lerobot.teleoperators.urdf_leader import UrdfLeader, UrdfLeaderConfig
from lerobot.processor.factory import make_default_processors
from lerobot.utils.control_utils import init_keyboard_listener
import os
import uuid
import rerun as rr
from lerobot.scripts.lerobot_record import record_loop

NUM_EPISODES = 5
FPS = 30
EPISODE_TIME_SEC = 60
RESET_TIME_SEC = 10
TASK_DESCRIPTION = "DemoRecording"
REPO_ID = "cnboonhan/embodied-ai-toolkit"

# camera_config = {"ego": OpenCVCameraConfig(index_or_path=0, width=640, height=480, fps=FPS)}
robot_config = UrdfFollowerConfig(
    action_grpc_endpoint="localhost:5001",
    observation_grpc_endpoint="localhost:5002",
    motor_whitelist=["right_shoulder_pitch_joint"],
    cameras={}
)
teleop_config = UrdfLeaderConfig(
    grpc_endpoint="localhost:5000", 
    motor_whitelist=["right_shoulder_pitch_joint"]
)

teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

# Initialize the robot and teleoperator
robot = UrdfFollower(robot_config)
teleop = UrdfLeader(teleop_config)

# Configure the dataset features
action_features = hw_to_dataset_features(robot.action_features, "action")
obs_features = hw_to_dataset_features(robot.observation_features, "observation")
dataset_features = {**action_features, **obs_features}

# Create the dataset
dataset = LeRobotDataset.create(
    repo_id=REPO_ID,
    root=f"{os.path.expanduser('~')}/datasets/{TASK_DESCRIPTION}",
    fps=FPS,
    features=dataset_features,
    robot_type=robot.name,
    use_videos=True,
    image_writer_threads=4,
)

# Initialize the keyboard listener and rerun visualization
_, events = init_keyboard_listener()
rr.init('dataset_exploration', recording_id=uuid.uuid4())
rr.connect_grpc()

# Connect the robot and teleoperator
robot.connect()
teleop.connect()

episode_idx = 0
while episode_idx < NUM_EPISODES and not events["stop_recording"]:
    print(f"Recording episode {episode_idx + 1} of {NUM_EPISODES}")

    record_loop(
        teleop_action_processor=teleop_action_processor,
        robot_action_processor=robot_action_processor,
        robot_observation_processor=robot_observation_processor,
        robot=robot,
        events=events,
        fps=FPS,
        teleop=teleop,
        dataset=dataset,
        control_time_s=EPISODE_TIME_SEC,
        single_task=TASK_DESCRIPTION,
        display_data=True,
    )

    # Reset the environment if not stopping or re-recording
    if not events["stop_recording"] and (episode_idx < NUM_EPISODES - 1 or events["rerecord_episode"]):
        print("Reset the environment")
        record_loop(
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
            robot_observation_processor=robot_observation_processor,
            robot=robot,
            events=events,
            fps=FPS,
            teleop=teleop,
            control_time_s=RESET_TIME_SEC,
            single_task=TASK_DESCRIPTION,
            display_data=True,
        )

    if events["rerecord_episode"]:
        print("Re-recording episode")
        events["rerecord_episode"] = False
        events["exit_early"] = False
        dataset.clear_episode_buffer()
        continue

    dataset.save_episode()
    episode_idx += 1

# Clean up
print("Stop recording")
robot.disconnect()
teleop.disconnect()
dataset.push_to_hub()