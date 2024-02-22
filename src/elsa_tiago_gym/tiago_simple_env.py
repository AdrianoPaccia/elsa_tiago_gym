import rospy
import numpy as np
from gym import spaces
from gym.spaces import Discrete
import random
from gym.envs.registration import register
from elsa_tiago_gym import tiago_env
from gazebo_msgs.msg import ContactsState, ModelStates
from sensor_msgs.msg import Image
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
    
    def __init__(self,env_code=None,max_episode_steps = 128, multimodal=False, discrete=True):
        super(TiagoSimpleEnv, self).__init__(env_code)

        #self.env_code = env_code
        self.max_episode_steps = max_episode_steps

        # Observation space
        self.is_multimodal = multimodal
        self.obs_low = np.array([0.4, -0.25, 0.65])
        self.obs_high = np.array([0.76, 0.25, 0.75])
        self.observation_space = spaces.Box(self.obs_low, self.obs_high)

        #define which collions are not important
        self.items = list(self.model_state.cubes.keys())
        self.items.append("static_cube")
        self.forniture = list(self.model_state.cylinders.keys())
        self.forniture.append("table_0m4")
        self.collision_detected = False

        # Action spaces
        self.discrete_actions = discrete
        if self.discrete_actions:
            self.action_space = spaces.Discrete(7)
        else:
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

        # randomize goal
        x = np.random.uniform(self.obs_low[0], self.obs_high[0])
        y = np.random.uniform(self.obs_low[1], self.obs_high[1])
        z = np.random.uniform(self.obs_low[2], self.obs_high[2])

        x = self.obs_points['x'][np.abs(np.asarray(self.obs_points['x'])-x).argmin()]
        y = self.obs_points['y'][np.abs(np.asarray(self.obs_points['y'])-y).argmin()]
        z = self.obs_points['z'][np.abs(np.asarray(self.obs_points['z'])-z).argmin()]

        self.goal = [x, y, z]

        # get the training parameters
        self.reward_coef ={   
                'R_goal': 10,
                'R_coll': -10,
                'R_dist': -1.0,
                'R_semigoal': 0.0,
                'R_fail': 0.0,
                }
        
        rospy.Subscriber("/table_contact", ContactsState, self.collision_cb)
        rospy.Subscriber("/cylinder_contact", ContactsState, self.collision_cb)
        #rospy.Subscriber("/gazebo/model_states", ModelStates, self.orientation_controller)




    def init_environment(self, test_env):
        #To integrate the cube deletion loop
        model_states_msg = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=5.0)
        cube_names = [obj for obj in model_states_msg.name if obj[:4]=='cube']
        for cube in cube_names:
            self.delete_object(cube)
        rospy.sleep(2)
        
        number_of_cube = 0
        rospack = rospkg.RosPack()
        path_to_models = os.path.join(rospack.get_path('pal_gazebo_worlds'),'models')
        for cube in self.environments['environments'][test_env]:
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
        poses = [[0.65, -0.1, 0.85, np.radians(90), np.radians(90)/2, np.radians(90)]]
        self.execute_trajectory(poses,only_arm=False)
        self.arm_torso_group.clear_pose_targets()


    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        self.episode_step = 0
        self.collision_detected = False
        self.grasped_item = None
        self.placed_item = None
        self.action_failed = False
        for id,cube in self.model_state.cubes.items():
            cube_state = self.get_gazebo_object_state(id,'')
            cube_state.header.frame_id = "base_footprint"
            self.scene.add_box(id, cube_state, size=(cube.side+0.01, cube.side+0.01, cube.side+0.01))




    def _set_action(self, action):
        """
        Move the robot based on the action variable given
        4 dim vector composed of the 3 continuos displacements and a boolean for grasping.
        """
        self.episode_step += 1
        
        #position related action
        scale=0.05
        if type(action) == np.ndarray:
            action = action.tolist()

        dx, dy, dz, grasp = action
        x, y, z, _, _, _ = self.stored_arm_state

        grasp_item_id,distance = self.get_grasping_obj()
        # grasping/releasing action
        if grasp == True and grasp_item_id is not None:
            if self.grasped_item == None:
                # grasp the closest obstacle if it is in proximity
                if grasp_item_id is not None:
                    grasp_item = self.model_state.cubes[grasp_item_id]
                    grasping_accomplished = self.grasping(grasp_item)
                    if not grasping_accomplished:
                        rospy.logwarn('GRASPING attempted, NOT accomplished')
                    #else:
                    #    rospy.logwarn('GRASPING attempted and ACCOMPLISHED >> {:}'.format(grasp_item.id))
            else:
                #release what ever is the grasped object
                #rospy.logwarn('PLACING attempted and ACCOMPLISHED >> {:}'.format(self.grasped_item))
                placing_accomplished = self.placing()
        else:
            x_target = x + dx*scale
            y_target = y + dy*scale
            z_target = z + dz*scale

            roll, pitch, yaw = 0, np.radians(90), 0
            #if y_target>0:
            #    roll, pitch, yaw = np.radians(90), np.radians(90)/2, np.radians(90)
            #else:
            #    roll, pitch, yaw = np.radians(90), np.radians(90)*3/2, np.radians(90)

            self.visualize_points(x_target,y_target,z_target, ns='action')
            self.action_failed = not self.set_arm_torso_pose(x_target,y_target,z_target, roll, pitch, yaw)
            #if self.action_failed:
            #    rospy.logwarn('ACTION failed')



    def _get_obs(self):
        """
        Here we define what sensor data of our robots observations
        To know which Variables we have acces to, we need to read the
        MyRobotEnv API DOCS
        :return: observations
        """
        self.store_model_state()
        if self.collision_detected is False:
            x, y, z, _, _, _ = self.stored_arm_state
        else:
            x, y, z, _, _, _ = self.collision_arm_state
        gripper_pose = np.array([x, y, z])
        
        if self.is_multimodal:
            img_msg = rospy.wait_for_message("/xtion/rgb/image_raw", Image)
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
            image = np.array(cv_image)
            return [image,gripper_pose]
        else:
            cubes_obs, cylinders_obs = self.model_state.get_obs()
            return [cubes_obs,cylinders_obs,gripper_pose]

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
        Return the reward based on the observations given
        """
        reward = 0
        # add a reward if the goal is accomplished
        if self.model_state.all_cubes_in_cylinders() is True:
            reward += self.reward_coef['R_goal']
        # add a penalty if a collision is detected
        if self.collision_detected is True:
            reward += self.reward_coef['R_coll']
        # add a penalty if the command generates a not reachable position
        if (self.out_of_reach or self.action_failed) is True:
            reward += self.reward_coef['R_fail']
            self.out_of_reach = False
        # add a reward/penality for each cube placed in the rigth/wrong box
        reward += sum(self.model_state.cubes_in_cylinders()) * self.reward_coef['R_semigoal']
        # add a penaly for the distance of the cubes from the respective cylinders
        reward += sum(self.model_state.get_distances()) * self.reward_coef['R_dist']

        return reward
        
    # Internal TaskEnv Methods
    #------------------------------------------------

    # To extend for all objects in the different worlds
    def collision_cb(self, data):
        for contacts in data.states:
            coll_1 = contacts.collision1_name.split('::')
            coll_2 = contacts.collision2_name.split('::')
            c1 = coll_1[0]
            c2 = coll_2[0]
            if (c1 or c2) in self.items:
                if c1 == self.grasped_item: self.placed_item = self.grasped_item
                elif c2 == self.grasped_item: self.placed_item = self.grasped_item
                else: pass
            elif (c1 and c2) in [*self.items,*self.forniture] or self.collision_detected:
                pass
            else:
                self.collision_detected = True
                self.collision_arm_state = self.stored_arm_state
                rospy.logerr("COLLISION DETECTED between {:} - {:}".format(c1,c2))

    def get_grasping_obj(self):
        x, y, z, _, _, _ = self.stored_arm_state
        min_dist = 999
        candidate = 'None'
        for id,cube in self.model_state.cubes.items():
            x_c,y_c,z_c = cube.position
            dist = math.sqrt((x_c-x)**2+(y_c-y)**2+(z_c-z)**2)
            if dist < min_dist:
                min_dist = dist
                candidate = id
        if dist<0.3:
            return candidate, dist
        else:
            return None,None