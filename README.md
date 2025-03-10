# TECHIN516_turtlebot3_kinova_final
This is a repo for turtlebot3 and Kinova gen3_lite arm, for TECHIN516 in the GIX MSTI program.

How to get it working:
1. On your laptop run `fastdds discovery --server-id 0`
2. `ssh ubuntu@10.18.3.87`
3. On the robot, run: `ros2 launch turtlebot3_bringup robot.launch.py`
( you may launch this to ensure connection: `ros2 run turtlebot3_teleop teleop_keyboard`)
4. Run this to connect to the kinova arm
`ros2 launch kortex_bringup gen3_lite.launch.py \
robot_ip:=10.18.2.240 \
launch_rviz:=false`
5. Run this to launch rviz
`ros2 launch kinova_gen3_lite_moveit_config robot.launch.py \
robot_ip:=10.18.2.240`
6. Finally, run this: `ros2 run 516_final pick_place_move`


Other Info:
My public ip address: 10.18.35.114
Robot used in lab: turtle6

