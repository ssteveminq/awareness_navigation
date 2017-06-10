#!/usr/bin/env python
import rospy
import roslib
import actionlib
import control_msgs.msg
import controller_manager_msgs.srv
import trajectory_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Int8

def stringcallback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def intcallback(data):
	rospy.loginfo(rospy.get_caller_id()+"I heard %d",data.data)
    
    	command = data.data

	# if command ==1:
	#    num_cmd=22
 #    elif command ==2:
 #       num_cmd=11
 #    elif command ==3:
 #       num_cmd=11
 #    else:
 #       num_cmd=0

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/CBA_cmd_int", Int8, intcallback)

    cli = actionlib.SimpleActionClient('/hsrb/omni_base_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
    cli.wait_for_server()
    #rospy.Subscriber("chatter", String, callback)
    rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
	list_controllers = rospy.ServiceProxy('/hsrb/controller_manager/list_controllers', controller_manager_msgs.srv.ListControllers)
	running = False
	while running == False:
    	rospy.sleep(0.1)
    	for c in list_controllers().controller:
        	if c.name == 'omni_base_controller' and c.state == 'running':
            	running = True

    rospy.spin()

# fill ROS message
	goal = control_msgs.msg.FollowJointTrajectoryGoal()
	traj = trajectory_msgs.msg.JointTrajectory()
	traj.joint_names = ["odom_x", "odom_y", "odom_t"]
	p = trajectory_msgs.msg.JointTrajectoryPoint()
	p.time_from_start = rospy.Time(6) 
	p.positions = [1, 0, 0]
	p.velocities = [0, 0, 0]
	traj.points=[p]
	goal.trajectory =traj
	cli.send_goal(goal)
	cli.wait_for_result()


# publisher = rospy.Publisher("/cba/move_base_goal", MoveBaseActionGoal, queue_size=1)
if __name__ == '__main__':
    listener()































#!/usr/bin/env python
# import roslib
# import rospy
# import actionlib
# import control_msgs.msg
# import controller_manager_msgs.srv
# import trajectory_msgs.msg
# import geometry_msgs.msg
# import std_msgs.msg



















# rospy.init_node('test')

# # initialize action client
# cli = actionlib.SimpleActionClient('/hsrb/omni_base_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)

# # wait for the action server to establish connection
# cli.wait_for_server()

# # make sure the controller is running
# rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
# list_controllers = rospy.ServiceProxy('/hsrb/controller_manager/list_controllers', controller_manager_msgs.srv.ListControllers)
# running = False
# while running == False:
#     rospy.sleep(0.1)
#     for c in list_controllers().controller:
#         if c.name == 'omni_base_controller' and c.state == 'running':
#             running = True

# # fill ROS message
# goal = control_msgs.msg.FollowJointTrajectoryGoal()
# traj = trajectory_msgs.msg.JointTrajectory()
# traj.joint_names = ["odom_x", "odom_y", "odom_t"]
# p = trajectory_msgs.msg.JointTrajectoryPoint()
# p.positions = [1, 0, 0]
# p.velocities = [0, 0, 0]
# p.time_from_start = rospy.Time(6)

# # p2 = trajectory_msgs.msg.JointTrajectoryPoint()
# # p2.positions = [0, 1, 0]
# # p2.velocities = [0, 0, 0]
# # p2.time_from_start = rospy.Time(6)


# traj.points = [p]
# goal.trajectory = traj

# # send message to the action server
# cli.send_goal(goal)

# # wait for the action server to complete the order
# cli.wait_for_result()
