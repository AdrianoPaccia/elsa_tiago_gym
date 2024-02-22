#! /usr/bin/env python3
import rospy
import moveit_commander
import sys
import numpy as np

from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import GetModelState
from std_srvs.srv import Empty
from std_msgs.msg import ColorRGBA, Bool
from control_msgs.msg import PointHeadActionGoal


class TestEnv():
    """
    Observation:
        Type: Box(3)
        Num     Observation
        0       absolute pos of end effector
        1       relative pos of end effector to goal
        2       distance between gripper fingers

    Actions:
        Type: Box(4)
        Num     Action
        0       x-pos of end effector
        1       y-pos of end effector
        2       z-pos of end effector
        3       distance between gripper fingers

    Reward:

    """
    def __init__(self):
        self.collision_detected = False
        rospy.Subscriber("/table_contact", ContactsState, self.gcollision_cb)
        

    def gcollision_cb(self, data):
        # rospy.loginfo(data)
        for contacts in data.states:
            if (contacts.collision1_name or contacts.collision1_name) not in [
                "cube_blue::link::collision",
                "cylinder_blue::link::collision",
                "cube_yellow::link::collision",
                "cylinder_yellow::link::collision",
                "cube_red::link::collision",
                "cylinder_red::link::collision",
                "cube_green::link::collision",
                "cylinder_green::link::collision",
                "table_0m4::link::surface",
                ]:
                self.collision_detected = True
                rospy.logerr("COLLISION DETECTED")

        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

    def init_moveit(self):

        self.store_model_state()

        self.arm_torso_group = moveit_commander.MoveGroupCommander('arm_torso')
        self.arm_torso_group.set_planning_time(0.15)
        self.store_torso_state()

        self.gripper_group = moveit_commander.MoveGroupCommander('gripper')
        self.gripper_group.set_planning_time(0.1)
        self.store_gripper_state()

        self.arm_group = moveit_commander.MoveGroupCommander('arm')
        self.arm_group.set_planning_time(0.1)
        self.store_arm_state()

        rospy.loginfo(self.cube_blue)
        rospy.loginfo(self.cylinder_blue)
        rospy.loginfo(self.stored_gripper_state)
        rospy.loginfo(self.stored_torso_state)
        rospy.loginfo(self.stored_arm_state)
        self.start_octomap = rospy.ServiceProxy('/start_octomap_updater', Empty)
        self.stop_octomap = rospy.ServiceProxy('/stop_octomap_updater', Empty)
        self.pub_head_topic = rospy.Publisher(
            '/head_controller/point_head_action/goal', 
            PointHeadActionGoal, queue_size=1)
        # self.start_octomap()
        self.set_torso_joint(0.20)
        self.set_gripper_joints(0.04,0.04)
        self.look_down()
        rospy.sleep(2)

    def look_down(self):
        phag = PointHeadActionGoal()
        phag.header.frame_id = "/base_link"
        phag.goal.max_velocity = 1.0
        phag.goal.min_duration = rospy.Duration(0.2)

        phag.goal.target.header.frame_id = "base_link"
        phag.goal.target.point.x = 0.9
        phag.goal.target.point.z = 0.0

        phag.goal.pointing_axis.x = 1.0
        phag.goal.pointing_frame = "head_2_link"

        self.pub_head_topic.publish(phag)


    def store_arm_state(self):
        pose = self.arm_group.get_current_pose().pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        roll, pitch, yaw = self.arm_group.get_current_rpy()

        self.stored_arm_state = [x, y, z, roll, pitch, yaw]

    def store_gripper_state(self):
        self.stored_gripper_state = self.gripper_group.get_current_joint_values()

    def store_torso_state(self):
        self.stored_torso_state = (self.arm_torso_group.get_current_joint_values())[0]

    def store_model_state(self):
        get_gazebo_object_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.cube_blue = get_gazebo_object_state('cube_blue','')
        self.cylinder_blue = get_gazebo_object_state('cylinder_blue','')

    def set_torso_joint(self, height):
        pose = self.arm_torso_group.get_current_joint_values()
        pose[0] = height
        self.arm_torso_group.set_joint_value_target(pose)
        plan = self.arm_torso_group.go(wait=True)
        self.arm_torso_group.stop()
        self.store_torso_state()
        return plan

    def set_gripper_joints(self, l, r):
        self.gripper_group.set_joint_value_target([l, r])
        plan = self.gripper_group.go(wait=True)
        self.gripper_group.stop()
        self.store_gripper_state()
        return plan

    def grasping(self):
        roll, pitch, yaw = np.radians(90), 0, 0
        self.set_arm_pose(self.cube_blue.pose.position.x - 0.25, self.cube_blue.pose.position.y, self.cube_blue.pose.position.z + 0.08, roll, pitch, yaw)
        rospy.sleep(1)
        self.store_torso_state()
        self.set_torso_joint(self.stored_torso_state - 0.05)
        rospy.sleep(1)
        grasp = rospy.ServiceProxy('/parallel_gripper_controller/grasp', Empty)
        grasp()
        grasped = rospy.wait_for_message("/parallel_gripper/is_grasped", Bool)
        rospy.logwarn(grasped)
        # self.stop_octomap()
        self.store_torso_state()
        self.set_torso_joint(self.stored_torso_state + 0.05)
        rospy.sleep(1)
        if grasped.data is True:
            return True
        else:
            # self.start_octomap()
            test.set_gripper_joints(0.04,0.04)
            return False

    def set_arm_pose(self, x, y, z, roll, pitch, yaw):
        self.arm_group.set_pose_target([x, y, z, roll, pitch, yaw])

        plan = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        self.store_arm_state()
        return plan

    def store_arm_state(self):
        pose = self.arm_group.get_current_pose().pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        roll, pitch, yaw = self.arm_group.get_current_rpy()

        self.stored_arm_state = [x, y, z, roll, pitch, yaw]

if __name__ == '__main__':
    rospy.init_node('env_test')
    test = TestEnv()
    test.init_moveit()
    roll, pitch, yaw = np.radians(90), 0, 0
    test.set_arm_pose(0.39, 0.08, 0.8, roll, pitch, yaw)
    failure = test.grasping()
    rospy.logerr(failure)
    test.set_arm_pose(test.cylinder_blue.pose.position.x - 0.20, test.cylinder_blue.pose.position.y, test.cylinder_blue.pose.position.z + 0.20, roll, pitch, yaw)
    test.set_gripper_joints(0.04,0.04)
    rospy.spin()
