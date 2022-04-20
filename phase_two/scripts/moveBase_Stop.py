#!/usr/bin/env python2.7
from os import system
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

odom_x = 0.0
odom_y = 0.0

def movebase_client():
    system("rosnode kill explore")
	# Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

	# Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

	# Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
	# Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = odom_x
    goal.target_pose.pose.position.y = odom_y
	# No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

	# Sends the goal to the action server.
    client.send_goal(goal)
    client.cancel_all_goals()


if __name__ == '__main__':
    rospy.init_node('movebase_client_test_py')
    odom_initial_pose = rospy.wait_for_message("/odom", Odometry)
    # coords for the start position
    odom_x = odom_initial_pose.pose.pose.position.x
    odom_y = odom_initial_pose.pose.pose.position.y
    rate = rospy.Rate(10)

    movebase_client()

    rospy.spin()