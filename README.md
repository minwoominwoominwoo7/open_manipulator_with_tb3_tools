## 모델 셋팅 
eb 로 .bashrc 파일을 아래처럼 모델 설정 ,waffle 로 샛팅    
#export TURTLEBOT3_MODEL=burger  
export TURTLEBOT3_MODEL=waffle   
#export TURTLEBOT3_MODEL=waffle_pi    

## ///실행 명령어 ////  
roslaunch open_manipulator_with_tb3_gazebo home_service.launch   
roslaunch open_manipulator_with_tb3_tools rooms_mnp.launch use_platform:=false   
roslaunch open_manipulator_with_tb3_tools task_controller_mnp.launch   

## ///슬램 명령어 ////   
roslaunch open_manipulator_with_tb3_tools slam.launch use_platform:=false   
ROS_NAMESPACE=om_with_tb3 roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch   
ROS_NAMESPACE=om_with_tb3 rosrun map_server map_saver -f ~/map   

