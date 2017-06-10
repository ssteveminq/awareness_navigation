#!/usr/bin/env python
import roslib
import rospy
import trajectory_msgs.msg
import std_msgs.msg


rospy.init_node('human tracking cmd test')

# initialize ROS publisher
pub = rospy.Publisher('/Int_cmd_trackhuman', std_msgs.msg.Int8, queue_size=10)

# wait to establish connection between the controller
while pub.get_num_connections() == 0:
    rospy.sleep(0.1)
# make sure the controller is running
# fill ROS message
int_msg=std_msgs.msg.Int8()
int_msg.data=1

r = rospy.Rate(5) # 10hz
while not rospy.is_shutdown():
	pub.publish(int_msg)
	r.sleep()


# publish ROS message
