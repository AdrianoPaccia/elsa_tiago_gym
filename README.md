# 🦾 tiago_gym
A ROS package to run Reinforcement Learning experiments, particularly pick and place tasks, on the TIAGo robot. Uses Gazebo, Rviz and MoveIt! (for motion planning)

<p align="center">
  <img src="https://user-images.githubusercontent.com/28115337/128855778-1333fb2a-a6ac-47d0-8d59-ccc5798a2c32.gif" alt="rviz-showcase" />
</p>

## Installation
Tested on Ubuntu 18.04 only. Beware! Instructions assume familiarity with the ROS [packages](http://wiki.ros.org/Packages) system.
- Install ROS Melodic + TIAGo
    -  http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS
- Install openai_ros package into your TIAGo workspace
    ``` bash
     cd /home/user/tiago_public_ws/src
     git clone https://bitbucket.org/theconstructcore/openai_ros.git
     cd openai_ros;git checkout version2
     cd /home/user/tiago_public_ws;catkin_make;source devel/setup.bash
    ``` 
- Install tiago_gym_parallel package into your TIAGo workspace
    ``` bash
     cd /home/user/tiago_public_ws/src
     git clone https://github.com/AdrianoPaccia/tiago_gym_parallel.git
     cd /home/user/tiago_public_ws;catkin_make;source devel/setup.bash
    ``` 
- Launch an environment!
    - 'roslaunch tiago_gazebo tiago_gazebo.launch world:=elsa end_effector:=robotiq-2f-85 public_sim:=true '
    - Gazebo and Rviz should launch similarly to gif above.
    - To speed up simulation you can run the command `gz physics –u 0 -s 0.0025` in a separate terminal.

### TiagoSimpleEnv-v0
Describe the environment
