#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

import numpy as np

class TrajectoryPublisher:
    def __init__(self):
        rospy.init_node('trajectory_publisher', anonymous=True)
        self.pub = rospy.Publisher('/arm_velocity_trajectory_controller/command', JointTrajectory, queue_size=10)
        #self.pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
        self.sub = rospy.Subscriber('/target_velocity', Float32MultiArray, self.target_velocity_callback)
        self.sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        self.joint_names = ['arm_' + str(i) + '_joint' for i in range(1, 8)]
        self.target_velocity = np.zeros(7)  # Initialize with zeros
        joint_states_msg = rospy.wait_for_message('/joint_states', JointState, timeout=5.0)
        self.curr_position = np.array(joint_states_msg.position[:7])   
        self.curr_velocity = np.array(joint_states_msg.velocity[:7])   
        self.dt = 0.05
        self.rate = rospy.Rate(20)  # 20 Hz

    def target_velocity_callback(self, msg):
        """
        When a new target velocity is published, the target changes
        """
        self.target_velocity = [0]*7
        self.target_velocity[0] = msg.data
        print('Got a msg: ', self.target_velocity)
        
    def joint_states_callback(self,msg):
        """
        Update the curret position and velocity of the joint 
        """
        self.curr_position = np.array(msg.position[:7])
        self.curr_velocity = np.array(msg.velocity[:7])
        #print('curr_pos')
        

    def publish_trajectory(self):
        while not rospy.is_shutdown():
            msg = self.assemble_msg()  
            self.pub.publish(msg)
            self.rate.sleep()



    def assemble_msg(self):
        pos = [p0_i + v0_i * self.dt for v0_i, p0_i in zip( self.target_velocity, self.curr_position)]

        trajectory = JointTrajectory()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.header.frame_id = "world"
        trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = pos # Initial position
        point.velocities = [0.0]*7
        point.accelerations = [0.0]*7
        point.time_from_start = rospy.Duration(self.dt)  # Duration of one control cycle (1 / 20 = 0.05)

        trajectory.points.append(point)
        
        #self.curr_position = pos

        return trajectory


if __name__ == '__main__':
    try:
        trajectory_publisher = TrajectoryPublisher()
        trajectory_publisher.publish_trajectory()
    except rospy.ROSInterruptException:
        pass


