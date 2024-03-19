#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

def publish_trajectory():
    rospy.init_node('trajectory_publisher', anonymous=True)
    pub = rospy.Publisher('/arm_velocity_trajectory_controller/command', JointTrajectory, queue_size=10)
    rate = rospy.Rate(20)  # 20 Hz

    while not rospy.is_shutdown():
        trajectory = JointTrajectory()
        trajectory.header = Header()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.joint_names = ['arm_' + str(i) + '_joint' for i in range(1, 8)]

        point = JointTrajectoryPoint()
        point.positions = [0.0] * 7  # Initial position
        point.positions[4] =1
        point.velocities = [0.0] * 7
        point.velocities[3] = 0.5  # 0.05 radians/sec velocity for each joint
        point.time_from_start = rospy.Duration(0.05)  # Duration of one control cycle (1 / 20 = 0.05)

        trajectory.points.append(point)

        pub.publish(trajectory)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_trajectory()
    except rospy.ROSInterruptException:
        pass
