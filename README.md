# PDM Project



## Outlines

* The gazebo_pkg contains the whole environment and a four-wheel differential car.
  * velocity control:     /cmd_vel
* The racecar contains a ackerman model and a basic controller. 
  * velocity control: 	/racecar/<u>right_front</u>_wheel_velocity_controller/command
  * steer control:         /racecar/left_steering_hinge_position_controller/command



## Potential Issues

* cannot run python scripts
  * chmod +x ___.py
  * conda deactivate



## Environments

* Ubuntu 18.04
* ROS Melodic
* Gazebo 



## Dependencies

sudo apt-get install ros-melodic-gazebo-ros-*

sudo apt-get install ros-melodic-ros-control-*

sudo apt-get install ros-melodic-ros-controllers-*

sudo apt install ros-melodic-controller-manager

sudo apt install ros-melodic-joint-state-controller

sudo apt install ros-melodic-position-controllers

sudo apt install ros-melodic-effort-controllers

sudo apt install ros-melodic-joint-trajectory-controller

sudo apt install ros-melodic-rosbridge-server

sudo apt install ros-melodic-ackermann-msgs

sudo apt-get install libsdl-dev

sudo apt-get install libsdl-image1.2-dev



## Preparation

​	Create the ROS workspace

* mkdir catkin_ws && cd catkin_ws

* mkdir src && catkin_make

  Clone the repository

* cd /catkin_ws/src

* git clone ______________

* cd .. && catkin_make

* source ./devel/setup.bash




## Launch the file

* Launch the gazebo environment and four-wheel differential model
  * roslaunch gazebo_pkg race.launch
* Launch the ackerman model and a basic controller
  * roslaunch racecar_gazebo racecar.launch 
  * rosrun racecar_gazebo demo.py 



### Useful Websites

[ROS Wiki](http://wiki.ros.org/ROS/Tutorials)

[Gazebo Tutorials](http://gazebosim.org/tutorials)

[Basic Tutorials of Navigation(CN)](http://www.iflyros.com/courseInfo?id=95)



roslaunch map_server rviz_mymap.launch

rosrun rrt_occupancy_grid rrt_occupancy
