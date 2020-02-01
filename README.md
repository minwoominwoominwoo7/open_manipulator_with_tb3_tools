## ///실행 명령어 ////  
roslaunch open_manipulator_with_tb3_gazebo home_service.launch   
roslaunch open_manipulator_with_tb3_tools rooms_mnp.launch use_platform:=false   
roslaunch open_manipulator_with_tb3_tools task_controller_mnp.launch   

## ///슬램 명령어 ////   
roslaunch open_manipulator_with_tb3_tools slam.launch use_platform:=false   
ROS_NAMESPACE=om_with_tb3 roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch   
ROS_NAMESPACE=om_with_tb3 rosrun map_server map_saver -f ~/map   

