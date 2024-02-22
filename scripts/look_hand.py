#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 5/8/15

@author: sampfeiffer

look_hand.py contains... a class to look at
its own hand. It could be easily extended to look
at any tf link
"""
__author__ = 'sampfeiffer'

# System imports

# Local imports

# ROS imports
import rospy
import tf.transformations

# ROS messages imports
from control_msgs.msg import PointHeadActionGoal
from geometry_msgs.msg import PointStamped, Point

# Useful colors for prints
HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'

LOOK_TO_POINT_AS_GOAL_TOPIC = '/head_controller/point_head_action/goal'

class LookAtMyHand():
    """This class does stuff"""

    def __init__(self):
        # Topics
        rospy.loginfo("Setting publisher to " + LOOK_TO_POINT_AS_GOAL_TOPIC)
        self.pub_head_topic = rospy.Publisher(LOOK_TO_POINT_AS_GOAL_TOPIC, PointHeadActionGoal, queue_size=1)

        self.tf_l = tf.TransformListener()
        rospy.sleep(2.0)


    def run(self):
        r = rospy.Rate(10)
        phag = PointHeadActionGoal()
        phag.header.frame_id = "/base_link"
        phag.goal.max_velocity = 1.0
        phag.goal.min_duration = rospy.Duration(0.2)
        phag.goal.target.header.frame_id = "/base_link"
        phag.goal.pointing_axis.x = 1.0
        phag.goal.pointing_frame = "/head_2_link"
        while not rospy.is_shutdown():
            #link_to_follow = "/arm_7_link"
            gripper_link_right = "/gripper_right_inner_finger"
            gripper_link_left = "/gripper_left_inner_finger"

            # Get right link Tf
            ps_r = PointStamped()
            ps_r.header.stamp = self.tf_l.getLatestCommonTime("/base_link", gripper_link_right)
            ps_r.header.frame_id = gripper_link_right
            transform_ok = False
            while not transform_ok and not rospy.is_shutdown():
                try:
                    gripper_right_link_ps = self.tf_l.transformPoint("/base_link", ps_r)
                    transform_ok = True
                except tf.ExtrapolationException as e:
                    rospy.logwarn("Exception on transforming point... trying again \n(" + str(e) + ")")
                    rospy.sleep(0.01)
                    ps_r.header.stamp = self.tf_l.getLatestCommonTime("/base_link", gripper_link_right)

            # Get left link Tf
            ps_l = PointStamped()
            ps_l.header.stamp = self.tf_l.getLatestCommonTime("/base_link", gripper_link_left)
            ps_l.header.frame_id = gripper_link_left
            transform_ok = False
            while not transform_ok and not rospy.is_shutdown():
                try:
                    gripper_left_link_ps = self.tf_l.transformPoint("/base_link", ps_l)
                    transform_ok = True
                except tf.ExtrapolationException as e:
                    rospy.logwarn("Exception on transforming point... trying again \n(" + str(e) + ")")
                    rospy.sleep(0.01)
                    ps_l.header.stamp = self.tf_l.getLatestCommonTime("/base_link", gripper_link_left)

            # Get the mid Point
            target_point = Point()
            target_point.x = (gripper_right_link_ps.point.x + gripper_left_link_ps.point.x)/2
            target_point.y = (gripper_right_link_ps.point.y + gripper_left_link_ps.point.y)/2
            target_point.z = (gripper_right_link_ps.point.z + gripper_left_link_ps.point.z)/2
            phag.goal.target.point = target_point
            point =  [phag.goal.target.point.x, phag.goal.target.point.y,  phag.goal.target.point.z]
            #rospy.loginfo("Sending: {:}".format(str(phag)))
            rospy.loginfo("-\n Point {:} {:} {:}".format(point[0],point[1],point[2]))
            self.pub_head_topic.publish(phag)
            r.sleep()



if __name__ == '__main__':
    rospy.init_node('look_at_my_hand')
    node = LookAtMyHand()
    node.run()
    #rospy.spin()

    