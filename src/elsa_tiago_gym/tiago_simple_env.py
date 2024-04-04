import rospy
import numpy as np
from gym import spaces
from gym.spaces import Discrete
import random
from gym.envs.registration import register
from elsa_tiago_gym import tiago_env
from gazebo_msgs.msg import ContactsState, ModelStates
from sensor_msgs.msg import Image,JointState
from std_msgs.msg import Header
from cv_bridge import CvBridge
from elsa_tiago_gym.utils import Model
import math
from geometry_msgs.msg import Twist,Pose,Point,Quaternion
import tf
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
import yaml
import rospkg
import os
from std_msgs.msg import Bool
from elsa_tiago_fl.utils.utils import tic,toc
import logging 



    
class TiagoSimpleEnv(tiago_env.TiagoEnv):
    """
    Observation:
        Type: Box(3)
        Num     Observation
        0       for each cube in the scene: absolute position, code_type, held (bool)
        1       for each cylinder in the scene: absolute position, code_type
        2       end effector position 

    Actions:
        Type: Box(4)
        Num     Action
        0       delta x-pos of end effector
        1       delta y-pos of end effector
        2       delta z-pos of end effector
        3       Grasping action (True/False)
    """
    
    def __init__(self,env_code=None,speed=0.0025, max_episode_steps = 200, random_init=False,multimodal=False, discrete=True):
        super(TiagoSimpleEnv, self).__init__(env_code,speed,random_init)

        self.max_episode_steps = max_episode_steps
        self.motion_time = 0.1
        self.max_joint_vel = np.ones(7)*0.01


        # Observation space
        self.is_multimodal = multimodal
        self.obs_low = np.array([0.4, -0.25, 0.65])
        self.obs_high = np.array([0.76, 0.25, 0.8])
        self.observation_space = spaces.Box(self.obs_low, self.obs_high)

        #define which collions are not important
        self.items = list(self.model_state.cubes.keys())
        self.items.append("static_cube")
        self.forniture = list(self.model_state.cylinders.keys())
        self.forniture.append("table_0m4")
        self.collision_detected = False

        # Action spaces
        dpos_low = np.array([-1.0, -1.0, -1.0])
        dpos_high = np.array([1.0, 1.0, 1.0])
        dpos_space = spaces.Box(low=dpos_low, high=dpos_high, shape=(3,), dtype=np.float32) # Space for increments in position
        hold_space = spaces.Discrete(2)  # Space for the booleans for picking/droping act 
        self.action_space = spaces.Tuple((dpos_space, hold_space))

        self.obs_points = {'x': [], 'y': [], 'z':[]}
        for x in np.arange(self.obs_low[0], self.obs_high[0]+0.1, 0.1):
            for y in np.arange(self.obs_low[1], self.obs_high[1]+0.1, 0.1):
                for z in np.arange(self.obs_low[2], self.obs_high[2]+0.1, 0.1):
                    self.obs_points['x'].append(x)
                    self.obs_points['y'].append(y)
                    self.obs_points['z'].append(z)
        self.visualize_points(self.obs_points['x'], self.obs_points['y'], self.obs_points['z'], ns='obs')



        # get the training parameters
        self.reward_coef ={   
                'R_goal': 10,
                'R_coll': -10,
                'R_dist': -1.0,
                'R_semigoal': 0.0,
                'R_fail': 0.0,
                }

        #define topics to listen to
        rospy.Subscriber("/table_contact", ContactsState, self.collision_cb)
        rospy.Subscriber("/cylinder_contact", ContactsState, self.collision_cb)
        rospy.Subscriber("/gripper/is_grasped", Bool, self.check_grasp)





    def init_environment(self, env_code=None):
        # get which env to init
        if env_code == None:
            env_code = random.randint(1,10)     
        test_env = 'elsa_'+str(env_code)

        #To integrate the cube deletion loop
        model_states_msg = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=5.0)

        cube_names = [obj for obj in model_states_msg.name if obj[:4]=='cube']
        for cube in cube_names:
            self.delete_object(cube)
        rospy.sleep(2)
        
        number_of_cube = 0
        rospack = rospkg.RosPack()
        path_to_models = os.path.join(rospack.get_path('pal_gazebo_worlds'),'models')
        env_kind = 'environments_' + str(1)
        for cube in self.environments[env_kind][test_env]:
            number_of_cube = number_of_cube + 1 
            cube_pose = Pose()
            cube_pose.position.x = cube['positions'][0]
            cube_pose.position.y = cube['positions'][1]
            cube_pose.position.z = cube['positions'][2]
            f = open(path_to_models+'/cube_'+cube['color']+'/cube_'+cube['color']+'.sdf','r')
            sdff = f.read()
            self.spawn_object("cube_"+cube['color']+"_"+str(number_of_cube),sdff, "", cube_pose, "world")
            #rospy.loginfo("cube_"+cube['color']+"_"+str(number_of_cube))
            sdff = f.close()


    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        
        if self.random_init:
            print('set init pose (RANDOM)')
            # put the gripper in a random pose
            x, y, z  = [np.random.uniform(low, high) for low, high in zip([0.40,-0.3,0.75], [0.8,0.3,0.85])]
            self.set_arm_pose(x, y, z, 0, np.radians(90), 0)
        else:
            print('set init pose')
            x, y, z  = [0.5,-0.1,0.8]
            self.set_arm_pose(x, y, z, 0, np.radians(90), 0)
        self.release()
        rospy.sleep(2)
        self.store_arm_state()


    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        self.episode_step = 0
        self.collision_detected = False
        self.grasped_item = None
        self.placed_item = False
        self.action_failed = False

        #change environment if it is randomized
        if self.random_env:
            self.gazebo.unpauseSim()
            self.init_environment()
            self.init_model_states()
            self.gazebo.pauseSim()

        if self.random_init:
            #put the objects in a random position
            for cube in self.model_state.cubes:
                x, y, z  = [np.random.uniform(low, high) for low, high in zip([0.40,-0.3,0.44], [0.5,0.3,0.54])]
                self.set_obj_pos(cube,[x, y, z, 0, 0, 0])





    def _set_action(self, action):
        """
        Move the robot based on the action variable given:
        8 dim vector: 7 JOINT displacemets (continuos) and 1 (boolean) for grasping.
        """
        self.episode_step += 1

        #joint position-related act
        if type(action) == np.ndarray:
            action = action.tolist()

        djoint = action[:-1]
        grasp = action[-1]

        grasp_item_id,distance = self.get_grasping_obj()

        # grasping/releasing action
        if grasp == True and grasp_item_id is not None:
            if self.grasped_item == None:
                # grasp the closest obstacle if it is in proximity
                if grasp_item_id is not None:
                    grasp_item = self.model_state.cubes[grasp_item_id]
                    grasping_accomplished = self.grasping(grasp_item)
                else:
                    pass
            else:
                placing_accomplished = self.placing()
        
        # joint motion
        else:
            # We get the joint values from the group and change some of the values
            joint_goal = self.arm_group.get_current_joint_values()
            for i in range(7):
                joint_goal[i] += (djoint[i]*self.max_joint_vel[i])*self.motion_time
            self.action_failed = not self.set_arm_joint_pose(joint_goal, self.motion_time)


    def _get_obs(self):
        """
        Get the current observations as:
        models states and joint contfiguration (if not multimodal)
                or 
        curent image and joint configuration (if multimodal)    
        """
        self.store_model_state()
        if self.collision_detected is False:
            arm_state = np.array(self.stored_join_state) 
            #arm_state = np.array(self.stored_arm_pose[:3])# position of the EE 
        else:
            arm_state = np.array(self.collision_arm_state) #position of the EE where the collision happened

        if self.is_multimodal:
            img_msg = rospy.wait_for_message("/xtion/rgb/image_raw", Image)
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
            image = np.array(cv_image)
            return [image,arm_state]
        else:
            cubes_obs, cylinders_obs = self.model_state.get_obs()
            return [cubes_obs,cylinders_obs,arm_state]


    def _is_done(self, observations):
        """
        Decide if episode is done based on the model state and if there has been a collision
        """
        if self.episode_step >= self.max_episode_steps or self.model_state.all_cubes_in_cylinders() or self.collision_detected:
            self.gazebo.unpauseSim()
            grasped = rospy.wait_for_message("/gripper/is_grasped", Bool)
            if grasped is True:
                self.placing()
            self.gazebo.pauseSim()
            done= True
        else:
            done= False
        #print('done: ',done)
        return done


    def _compute_reward(self, observations, done):
        """
        Return the reward based on the observations given, as:
            - Goal reaching (reward)
            - Collision (penalty)
        """

        reward =0 
        # add a reward if the goal is accomplished
        if self.model_state.all_cubes_in_cylinders() is True:
            reward += self.reward_coef['R_goal']
        # add a penalty if a collision is detected
        if self.collision_detected is True:
            reward += self.reward_coef['R_coll']        
        # add a reward for each cube placed in the rigth
        subgoals_old = self.model_state.cubes_subgoals
        subgoals_new = self.model_state.check_subgoals()
        delta_subgoals = [1 for old,new in zip(subgoals_old,subgoals_new) if (old==False and new == True)]
        reward += sum(delta_subgoals) * self.reward_coef['R_semigoal']
        # add a penaly for the distance of the cubes from the respective cylinders
        #reward += sum(self.model_state.get_distances()) * self.reward_coef['R_dist']
        return reward


        
    # Internal TaskEnv Methods
    #------------------------------------------------

    # TODO: I a collision I need to store the join values (not the gripper pose)
    def collision_cb(self, data):
        for contacts in data.states:
            coll_1 = contacts.collision1_name.split('::')
            coll_2 = contacts.collision2_name.split('::')
            c1 = coll_1[0]
            c2 = coll_2[0]
            if (c1 or c2) in self.items:
                if c1 == self.grasped_item or c2 == self.grasped_item: self.placed_item = True
                else: pass
            elif (c1 and c2) in [*self.items,*self.forniture] or self.collision_detected:
                pass
            else:
                self.collision_detected = True
                self.collision_arm_state = self.stored_join_state
                #self.collision_arm_state = self.stored_arm_pose[:3] #position where the collistion happened
                rospy.loginfo("COLLISION DETECTED between {:} - {:}".format(c1,c2))

            
    def check_grasp(self,data):
        if data is False and self.grasped_item is not None:
            self.grasped_item = None


    def get_grasping_obj(self):
        x, y, z, _, _, _ = self.stored_arm_pose
        min_dist = 999
        candidate = 'None'
        for id,cube in self.model_state.cubes.items():
            x_c,y_c,z_c = cube.position
            dist = math.sqrt((x_c-x)**2+(y_c-y)**2+(z_c-z)**2)
            if dist < min_dist:
                min_dist = dist
                candidate = id
        if dist<0.35 and z_c<z+0.25:
            return candidate, dist
        else:
            return None,None