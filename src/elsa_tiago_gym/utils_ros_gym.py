import rospy
import gym
import os
import rospkg
import sys
from gym.envs.registration import register

rospack = rospkg.RosPack()

def start_env(env, speed = None,client_id = None, max_episode_steps:int = 100, multimodal = False, ros_uri = "http://localhost:11311/", gz_uri='http://localhost:11312'):

    os.environ['ROS_MASTER_URI'] = ros_uri
    os.environ['GAZEBO_MASTER_URI'] = gz_uri
    setup_env(env,max_episode_steps)
    rospy.init_node('parallelSimulationNode',log_level=rospy.ERROR)
    env = gym.make(id=env,
                    env_code=client_id,
                    speed = speed,
                    max_episode_steps=max_episode_steps,
                    multimodal = multimodal
                    )
    return env




def setup_env(env_name, max_episode_steps):

    dir = rospack.get_path('elsa_tiago_gym')
    tasks_path = os.path.join(dir,'src/elsa_tiago_gym')
    sys.path.append(tasks_path)

    if env_name =='TiagoReachEnv-v0':
        entry_point = 'tiago_reach_env:TiagoReachEnv'
    elif env_name == 'TiagoSimpleEnv-v0':
        entry_point = 'tiago_simple_env:TiagoSimpleEnv'
    else:
        raise ValueError(
        "Environment '{:s}' not expected.".format(env_name))
    register(
        id=env_name, 
        entry_point=entry_point,
        max_episode_steps=max_episode_steps, 
    )
