# Humanoid Controller Simulation Examples

## AgiBot
```bash
# Agibot is closed source, URDF not available online for genie_sim. You have to source it yourself
# Clone Repo and Build GenieSim Docker Container: https://agibot-world.com/sim-evaluation/docs/#/v2
# Download Assets into genie_sim folder: https://huggingface.co/datasets/agibot-world/GenieSimAssets
# Build docker containers: https://agibot-world.com/sim-evaluation/docs/#/v2?id=_231-docker-container-recommended
# Run Simulation: https://agibot-world.com/sim-evaluation/docs/#/v2?id=_31-how-to-run-simulation-with-just-one-line-of-code

python3 src/main.py --path agibot-model.urdf --vis-port 8080 --api-port 5000 --ik-targets "arm_left_link7,arm_right_link7" --custom-joints "left_hand:0:2,right_hand:0:2"

cd ../genie_sim; SIM_ASSETS=./GenieSimAssets ./scripts/start_gui.sh; cd -
cd ../genie_sim; ./scripts/autorun.sh genie_task_cafe_espresso; cd -
docker cp src/joint_state_adapter.py genie_sim_benchmark:/tmp/; docker cp src/joint_state_adapter_ros2.py genie_sim_benchmark:/tmp/ 

docker exec -it genie_sim_benchmark bash
python3 /tmp/joint_state_adapter_ros2.py --server-url http://172.17.0.1:5000/get_joints --joint-subscription-topic joint_states --joint-publisher-topic /joint_command --history-length 2 --joint-name-mapping 'idx05_left_arm_joint1:idx21_arm_l_joint1:true,idx12_right_arm_joint1:idx61_arm_r_joint1:true,idx06_left_arm_joint2:idx22_arm_l_joint2:true,idx13_right_arm_joint2:idx62_arm_r_joint2:true,idx07_left_arm_joint3:idx23_arm_l_joint3:true,idx14_right_arm_joint3:idx63_arm_r_joint3:false,idx08_left_arm_joint4:idx24_arm_l_joint4:true,idx15_right_arm_joint4:idx64_arm_r_joint4:true,idx09_left_arm_joint5:idx25_arm_l_joint5:false,idx16_right_arm_joint5:idx65_arm_r_joint5:false,idx10_left_arm_joint6:idx26_arm_l_joint6:true,idx17_right_arm_joint6:idx66_arm_r_joint6:true,idx11_left_arm_joint7:idx27_arm_l_joint7:false,idx18_right_arm_joint7:idx67_arm_r_joint7:false,idx01_waist_lift_joint:idx01_body_joint1:false,idx02_waist_pitch_joint:idx02_body_joint2:false,left_hand:idx41_gripper_l_outer_joint1:true,right_hand:idx81_gripper_r_outer_joint1:true'

omni_python source/geniesim/teleop/teleop.py --task_name genie_task_cafe_espresso --mode keyboard
```