# 🦾 tiago_gym
A ROS package to run Reinforcement Learning experiments, particularly pick and place tasks, on the TIAGo robot.
It allows to run multiple instances of ROS and Gazebo running in parallel, so that are accessible by multiple parallel process. It is suggested to use of workers to collect exeperience, while another process updates the AI in parallel (it implies the use of off-policy algorithms).

![Alt Text](media/tiago_video.gif)

## Installation
- Install ROS Melodic + TIAGo
    -  http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS
- Install openai_ros package into your TIAGo workspace
    ``` bash
     cd /home/user/tiago_public_ws/src
     git clone https://bitbucket.org/theconstructcore/openai_ros.git
     cd openai_ros;git checkout version2
     cd /home/user/tiago_public_ws;catkin_make;source devel/setup.bash
    ``` 
- Install elsa_tiago_gym package into your TIAGo workspace
    ``` bash
     cd /home/user/tiago_public_ws/src
     git clone https://github.com/AdrianoPaccia/elsa_tiago_gym.git
     cd /home/user/tiago_public_ws;catkin_make;source devel/setup.bash
    ``` 
- Launch an environment!
    - 'roslaunch tiago_gazebo tiago_gazebo.launch world:=elsa end_effector:=robotiq-2f-85 public_sim:=true '
    - Gazebo and Rviz should launch similarly to gif above.
    - To speed up simulation you can run the command `gz physics –u 0 -s 0.0025` in a separate terminal.

### TiagoSimpleEnv-v0
TODO: Describe the environment
