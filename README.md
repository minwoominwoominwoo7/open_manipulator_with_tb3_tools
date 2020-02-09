## 설치 관련      
기본 로보티즈 래포지토리 중 아래 것들은    
https://github.com/minwoominwoominwoo7   해당 깃에 래파지토리로 변경.   

open_manipulator_with_tb3_simulations   
open_manipulator_with_tb3_tools   
open_manipulator_perceptions   
turtlebot3   

현재 기능 추가로 아래 프로그램 추가 설치해야함.    
sudo apt-get install ros-kinetic-uvc-camera       
   
그리퍼쪽 카메라를 위한 디팬더시 추가 설치 . 현재 해당기능 적용은 안되지만 디팬더시가 설정되어 설치해야 빌드가 됨.      
sudo apt-get install ros-kinetic-pcl-conversions    
sudo apt-get install ros-kinetic-pcl-ros      

## 모델 샛팅 현재 와플파이로 설정해서 URDF 수정했음.   
모델을 와플 파이로 설정해야함.    
eb 명령어를 통해 .bashrc 파일 아래 처럼 설정   
```bash
#export TURTLEBOT3_MODEL=burger
#export TURTLEBOT3_MODEL=waffle
export TURTLEBOT3_MODEL=waffle_pi
```

## ///가제보 기반 실행 명령어 ////  
```bash
roslaunch open_manipulator_with_tb3_gazebo home_service.launch   
roslaunch open_manipulator_with_tb3_tools rooms_mnp.launch use_platform:=false   
roslaunch open_manipulator_with_tb3_tools task_controller_mnp.launch 
```

## ///가제보 기반 슬램 명령어 ////  
```bash
roslaunch open_manipulator_with_tb3_tools slam.launch use_platform:=false   
ROS_NAMESPACE=om_with_tb3 roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch   
ROS_NAMESPACE=om_with_tb3 rosrun map_server map_saver -f ~/map   
```

## ///실물 기반 실행 명령어 ////  
참고    
http://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#bringup    
만일 USB 카메라 device가 video1이 아닌 0으로 잡혀 있다면 turtlebot3_usbcamera.launch 에서 해당 부분 변경 필요 
```bash
ROS_NAMESPACE=om_with_tb3 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:=om_with_tb3 set_lidar_frame_id:=om_with_tb3/base_scan
roslaunch open_manipulator_with_tb3_tools turtlebot3_usbcamera.launch
ROS_NAMESPACE=om_with_tb3 roslaunch open_manipulator_with_tb3_tools om_with_tb3_remote.launch
roslaunch open_manipulator_with_tb3_tools rooms_mnp.launch   
roslaunch open_manipulator_with_tb3_tools task_controller_mnp.launch   
``` 
## ///실물 슬램 명령어 ////   
참고     
http://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#slam       

