
import os
import multiprocessing as mp
from gym.envs.registration import register

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState,SetModelState
from tiago_gym.utils import setup_env
import gym

numberOfCpuCore_to_be_used = 2


class Worker(mp.Process):
    def __init__(self, someGlobalNumber, name):
        super(Worker, self).__init__()
        self.name = 'w%i' % name
        self.port = name
        self.global_number = someGlobalNumber


    def run(self):
        # get connected to one of the ros ports 
        os.environ['ROS_MASTER_URI'] = "http://localhost:1135" + str(self.port) + '/'
        rospy.init_node('parallelSimulationNode')


class Worker(mp.Process):
    def __init__(self, worker_id, replay_buffer, shared_policy, env_name, client_id):
        super(Worker, self).__init__()
        self.worker_id = worker_id
        self.replay_buffer = replay_buffer
        self.shared_policy = shared_policy
        self.env_name = env_name
        self.client_id = client_id

    def run(self):
        # get connected to one of the ros ports 
        os.environ['ROS_MASTER_URI'] = + str(self.port) + '/'
        rospy.init_node('parallelSimulationNode')

        self.env = gym.make(id=env,env_code=self.client_id,max_episode_steps=128)

        env = gym.make(self.env)  
        policy = self.shared_policy.value  # Initial policy

        while True:
            # Explore the environment using the current policy
            observation = env.reset()
            done = False
            while not done:
                action = self.select_action(policy, observation)
                next_observation, reward, done, _ = env.step(action)
                experience = {
                    'observation': observation,
                    'action': action,
                    'reward': reward,
                    'next_observation': next_observation,
                    'done': done
                }
                self.replay_buffer.append(experience)
                observation = next_observation

            # Update the policy if a new one is available
            new_policy = self.shared_policy.value
            if new_policy is not None:
                policy = new_policy
                self.shared_policy.value = None  # Reset shared_policy after updating

    @staticmethod
    def select_action(policy, observation):
        # Your logic for selecting an action using the policy tensor
        # Example: Assuming a simple feedforward network
        with torch.no_grad():
            observation_tensor = torch.tensor(observation, dtype=torch.float32)
            action_probs = policy(observation_tensor)
            action = torch.argmax(action_probs).item()
        return action


def call_multiprocess_training(env_name,
                               policy,
                               n_workers=2):
    
    # register the environment
    register(
        id=env_name, 
        entry_point='tiago_simple_env:TiagoSimpleEnv',
        max_episode_steps=100, 
    )
    replay_buffer = dict()  
    shared_policy = mp.Array('d', policy.state_dict().values())

    # Get all workers that collect experience
    workers = [Worker(i, replay_buffer, shared_policy) for i in range(n_workers)]

    # Start worker processes
    for worker in workers:
        worker.start()

    # Start the policy updater process that simultaneusly trains
    updater_process = mp.Process(target=policy_updater, args=(shared_policy,))
    updater_process.start()
    updater_process.join()






