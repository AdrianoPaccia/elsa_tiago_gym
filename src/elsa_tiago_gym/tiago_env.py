#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import numpy as np
from openai_ros import robot_gazebo_env
from openai_ros.openai_ros_common import ROSLauncher
from geometry_msgs.msg import PoseStamped, Point,Twist,Quaternion, Pose
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from std_msgs.msg import ColorRGBA, Bool,Header
from std_srvs.srv import Empty
from elsa_tiago_fl.utils.utils import tic,toc
from elsa_tiago_gym.utils_parallel import set_sim_velocity

from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import GetModelState,SetModelState
from control_msgs.msg import PointHeadActionGoal
from gazebo_msgs.msg import ModelStates,ModelState
from elsa_tiago_gym.utils import Model,objects_from_scene
from tf.transformations import quaternion_from_euler
import tf
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
import yaml
import rospkg
import os
import random
import time



class TiagoEnv(robot_gazebo_env.RobotGazeboEnv):
    """
    Initializes a new Tiago Steel environment
    """
    def __init__(self, env_code:str,speed:float,random_init:bool):
        rospy.logdebug("========= In Tiago Env")
        self.env_kind = 'environments_0'

        
        if env_code == None:
            self.random_env = True
        else:
            self.random_env = False

        self.ros_master_uri = os.environ.get('ROS_MASTER_URI')
        self.controllers_list = []
        self.robot_name_space = ""
        self.out_of_reach = True
        self.release = rospy.ServiceProxy('/gripper_controller/release', Empty)
        self.get_gazebo_object_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.pub_head_topic = rospy.Publisher(
            '/head_controller/point_head_action/goal', 
            PointHeadActionGoal, queue_size=1)
        self.pub_base_controller = rospy.Publisher('/mobile_base_controller/cmd_vel',Twist,queue_size=1)
        self.pub_arm_joint_controller = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)


        # Whether to reset controllers when a new episode starts
        reset_controls_bool = False
        
        # Parent class init
        super(TiagoEnv, self).__init__(controllers_list=self.controllers_list,
                                       robot_name_space=self.robot_name_space,
                                       reset_controls=reset_controls_bool,
                                       reset_world_or_sim="WORLD")

        self.grasp = rospy.ServiceProxy('/gripper_controller/grasp', Empty)
        

        # define services for changing environemnt
        self.delete_object = rospy.ServiceProxy('/gazebo/delete_model',DeleteModel)
        self.spawn_object = rospy.ServiceProxy('/gazebo/spawn_sdf_model',SpawnModel)
        rospack = rospkg.RosPack()
        path_to_yaml = os.path.join(rospack.get_path('elsa_tiago_gym'),'config/environment_variables.yaml')
        with open(path_to_yaml,'r') as file:
            try:
                self.environments = yaml.safe_load(file)
            except yaml.YAMLError as exc:
                print(exc)        
        self.random_init = random_init
        print('random init = ', self.random_init)


        # define the reachability boundaries (ws, joint bounds)
        self.joint_names = ["arm_"+str(i)+"_joint" for i in range(1,8)] 
        self.arm_workspace_low =  np.array([0.4, -0.5, 0.63])
        self.arm_workspace_high = np.array([0.8,  0.5, 0.9])
        #self.arm_joint_bounds_low = np.array( [60, 0, -90,  20, -120, -90, -120])/180 * np.pi 
        #self.arm_joint_bounds_high = np.array([120, 50,  90, 135, 120,  90,  120])/180 * np.pi
        self.arm_joint_bounds_low = np.array( [60, 0, -90,  20, -120, -45, -120])/180 * np.pi 
        self.arm_joint_bounds_high = np.array([120, 50,  90, 120, 120,  45,  120])/180 * np.pi

        # init and start
        self.gazebo.unpauseSim()
        self.init_environment(env_code)
        self.init_model_states()
        self._init_moveit()
        self._init_rviz()
        self._check_all_systems_ready()

        self.gazebo.pauseSim()

        if speed is not None:
            set_sim_velocity(speed)


    # Methods needed by the RobotGazeboEnv
    # ----------------------------
    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """

    # TiagoEnv virtual methods
    # ----------------------------

    def _init_moveit(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.arm_torso_group = moveit_commander.MoveGroupCommander('arm_torso')
        self.arm_torso_group.set_planning_time(1)

        self.arm_group = moveit_commander.MoveGroupCommander('arm')
        self.arm_group.set_planning_time(1)
        self.store_arm_state()

        rospy.sleep(2)
        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = 0.8
        p.pose.position.y = 0
        p.pose.position.z = 0.2

        self.arm_torso_group.go([0.3, 1.05, 0.3, -1.57, 2.0, 0.24, -0.5, 0.0],wait=True)
        self.arm_torso_group.set_pose_target([0.5,-0.1,0.8, 0, np.radians(90), 0])
        plan = self.arm_torso_group.go(wait=True)
        self.arm_torso_group.stop()
        self.arm_torso_group.clear_pose_targets()
        self.store_arm_state()
        #self.arm_torso_group.go([0.15, 1.0, 0.4, -0.7, 1.9, 0.3, -0.7, -0.5],wait=True)
        self.arm_torso_group.stop()
        rospy.sleep(2)

        '''        
        ## ADD objects in the scene
        # Place the table in movit
        self.table = self.get_gazebo_object_state('table_0m4','')
        self.table.header.frame_id = "base_footprint"
        self.table.pose.position.z = 0.21
        self.scene.add_box("table", self.table, size=(1.05, 0.85, 0.5))
        # Place the cubes and cylinders in movit
        
        for id,cube in self.model_state.cubes.items():
            cube_state = self.get_gazebo_object_state(id,'')
            cube_state.header.frame_id = "base_footprint"
            self.scene.add_box(id, cube_state, size=(cube.side+0.01, cube.side+0.01, cube.side+0.01))
        for id,cyl in self.model_state.cylinders.items():
            cyl_state = self.get_gazebo_object_state(id,'')
            cyl_state.header.frame_id = "base_footprint"
            self.scene.add_cylinder(id, cyl_state, cyl.heigth+0.02, cyl.radius+0.01)
        '''

    def _init_rviz(self):
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=100, latch=True)

    def init_model_states(self):
        self.grasped_item = None
        self.tiago_orientation = 0.0
        model_states_msg = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=5.0)
        model_names = model_states_msg.name 
        self.model_state = objects_from_scene(model_names)


    # Methods that the TrainingEnvironment will need.
    # ----------------------------
    def visualize_points(self, x, y, z, ns=''):
        marker = Marker()
        marker.header.frame_id = 'base_footprint'
        marker.type = marker.SPHERE_LIST
        marker.action = marker.ADD
        marker.ns = ns
        marker.id = 0

        if ns == 'goal':
            marker.color.g = 1.0
        elif ns == 'action':
            marker.color.b = 1.0

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        if type(x) == list:
            for x, y, z in zip(x, y, z):
                marker.points.append(Point(x, y, z-0.3))
        else:
            marker.points.append(Point(x, y, z-0.3))

        marker.color.a = 0.5

        self.marker_publisher.publish(marker)

    def visualize_action(self, prev_x, prev_y, prev_z, x, y, z, valid):
        marker = Marker()
        marker.header.frame_id = 'base_footprint'
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.ns = 'action'
        marker.id = 0

        marker.scale.x = 0.01
        marker.scale.y = 0.02

        if valid:
            marker.color.a = 1
            marker.color.b = 1
        else:
            marker.color.a = 0.5

        marker.points.append(Point(prev_x, prev_y, prev_z-0.3))
        marker.points.append(Point(x, y, z-0.3))

        self.marker_publisher.publish(marker)

    def look_down(self):
        phag = PointHeadActionGoal()
        phag.header.frame_id = "base_footprint"
        phag.goal.max_velocity = 1.0
        phag.goal.min_duration = rospy.Duration(0.2)

        phag.goal.target.header.frame_id = "base_footprint"
        phag.goal.target.point.x = 0.9
        phag.goal.target.point.z = 0.0

        phag.goal.pointing_axis.x = 1.0
        phag.goal.pointing_frame = "head_2_link"

        self.pub_head_topic.publish(phag)

    def grasping(self, obj):
        """
        1. remove the object from its place
        2. brings the EE to its position
        3. open the gripper
        4. put the obj in place again
        3. close the gripper
        """

        #collect the object position
        x,y,z = obj.position
        roll, pitch, yaw = 0, np.radians(90), 0
        self.release()
        '''
        # go over the object
        self.set_arm_pose(x, y, z + 0.27, roll, pitch, yaw)
        #remove the object
        self.set_obj_pos(obj.id,[10, 0, 0,0, 0, 0])
        #go in the object position
        self.set_arm_pose(x, y, z + 0.21, roll, pitch, yaw)
        '''
        self.execute_trajectory(
            [[x, y, z + 0.27, roll, pitch, yaw],
            [x, y, z + 0.23, roll, pitch, yaw]]
        )
        
        rospy.sleep(1)

        # get the object in position and grasp
        self.set_obj_pos(obj.id,[x, y, z, 0, 0, 0])
        self.grasp()
        grasped = rospy.wait_for_message("/gripper/is_grasped", Bool)

        if grasped.data is True:
            grasping_group = 'gripper'
            touch_links = self.robot.get_link_names(group=grasping_group)
            self.scene.attach_box("gripper_base_link", obj.id, touch_links=touch_links)
            self.grasped_item = obj.id
            self.set_arm_pose(x, y, z + 0.3, roll, pitch, yaw)
            return True
        else:
            self.release()
            self.grasped_item = None
            self.set_arm_pose(x, y, z + 0.3,roll, pitch, yaw)
            return False

    def placing(self):
        """
        1. Open the gripper
        2. Detach the object from the gripper group
        3. Wait for the object to fall
        """
        self.release()
        self.scene.remove_attached_object("gripper_base_link", name=self.grasped_item)
        placed = self.wait_for_placed_item(self.grasped_item)
        object_state = self.get_gazebo_object_state(self.grasped_item,'')
        object_state.header.frame_id = "base_footprint"
        '''
        self.scene.add_box(self.grasped_item, object_state, size=(object.side+0.01, object.side+0.01, object.side+0.01))
        '''
        self.grasped_item = None
        return True
    
    def wait_for_placed_item(self,item):
        self.placed_item = False
        self.to_place_item = item
        cnt=0
        while not self.placed_item:
            rospy.sleep(0.1)
            rospy.loginfo("waiting for dropping")
            cnt+=1
            if cnt>30:
                x, y, z, _, _,_ = self.stored_arm_pose
                self.set_obj_pos(self.grasped_item,[x, y, z-0.23,0, 0, 0])
                break
        return True

    def set_arm_pose(self, x, y, z, roll, pitch, yaw):
        """
        Motions of the arm in the cartesian space.
        """
        #if not self.arm_pose_reachable(x, y, z):
        #    self.out_of_reach = True
        #    return False

        x,y,z = np.clip([x,y,z], self.arm_workspace_low, self.arm_workspace_high)

        #print('setting: ',[x, y, z, roll, pitch, yaw])

        '''self.out_of_reach = False
        self.arm_group.set_pose_target([x, y, z, roll, pitch, yaw])
        plan = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        self.store_arm_state()'''
        self.out_of_reach = False
        self.arm_torso_group.set_pose_target([x, y, z, roll, pitch, yaw])
        plan = self.arm_torso_group.go(wait=True)
        self.arm_torso_group.stop()
        self.arm_torso_group.clear_pose_targets()
        self.store_arm_state()
        return plan

    def execute_trajectory(self,poses):
        '''waypoints = [ Pose(Point(*p[:3]), Quaternion(*quaternion_from_euler(*p[3:]))) for p in poses]
        (plan, fraction) = self.arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.arm_group.execute(plan, wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()'''
        waypoints = [ Pose(Point(*p[:3]), Quaternion(*quaternion_from_euler(*p[3:]))) for p in poses]
        (plan, fraction) = self.arm_torso_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.arm_torso_group.execute(plan, wait=True)
        self.arm_torso_group.stop()
        self.arm_torso_group.clear_pose_targets()
        self.store_arm_state()

    def set_arm_joint_pose(self, joint_goal, motion_time):
        """
        Motions of the arm in the joint space (7 joints)
        """
        if not self.arm_joints_feasible(joint_goal):
            print('not feasible')
            self.out_of_reach = True
            return False
        joint_goal  = np.clip(joint_goal, self.arm_joint_bounds_low, self.arm_joint_bounds_high)
        plan = self.arm_group.go(joint_goal,wait=True)
        self.arm_group.stop()
        
        self.store_arm_state()
        return plan 

    def store_arm_state(self):
        # store the arm cartesian poseget_current_pose
        pose = self.arm_group.get_current_pose().pose
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        roll, pitch, yaw = self.arm_group.get_current_rpy()
        self.stored_arm_pose = [x, y, z, roll, pitch, yaw]

    def arm_pose_reachable(self, x, y, z):
        return (([x, y, z] >= self.arm_workspace_low) & ([x, y, z] <= self.arm_workspace_high)).all()  

    def arm_joints_feasible(self,joints):
        return ((joints >= self.arm_joint_bounds_low) & (joints <= self.arm_joint_bounds_high)).all()

    def store_model_state(self):
        """
        To update the position of the objects in the scene
        """
        # upadte the model states
        get_gazebo_object_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        for id,cube in self.model_state.cubes.items():
            cube_state = get_gazebo_object_state(id,'')
            c = cube_state.pose.position
            if c.z < 0.4:
                self.set_obj_pos(id,[*cube.init_position,0,0,0])
            cube.set_state(cube_state)
            cube.held = True if id == self.grasped_item else False

        for id,cyl in self.model_state.cylinders.items():
            cyl_state = get_gazebo_object_state(id,'')
            cyl.set_state(cyl_state)

        attached_objects = self.scene.get_attached_objects(["gripper_base_link"])

    def set_obj_pos(self,id:str,pose):
        """
        To artiifcially change the position of the bjects in the scene
        """
        pos = pose[:3]
        euler = pose[3:]
        quaternion =quaternion_from_euler(*euler)
        state_msg = ModelState()
        state_msg.model_name = id
        state_msg.pose.position.x = pos[0]
        state_msg.pose.position.y = pos[1]
        state_msg.pose.position.z = pos[2]
        state_msg.pose.orientation.x = quaternion[0]
        state_msg.pose.orientation.y = quaternion[1]
        state_msg.pose.orientation.z = quaternion[2]
        state_msg.pose.orientation.w = quaternion[3]
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state(state_msg)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
    

    # Methods that the TrainingEnvironment will need to define here as virtual
    # because they will be used in RobotGazeboEnv GrandParentClass and defined in the
    # TrainingEnvironment.
    # ----------------------------
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()
    
    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()
    
    def get_grasping_obj(self):
        """Checks which is the closest grapable object
        """
        raise NotImplementedError()

    def init_environment(self,test_env):
        """Checks which is the closest grapable object
        """
        raise NotImplementedError()







