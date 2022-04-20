#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

lin_x = -1
ang_z = -1
odom_x = 0.0
odom_y = 0.0
num_of_instances = 200

def vel_callback(msg):
    '''
    Updates the linear and angular velocity.
    '''
    global lin_x, ang_z
    lin_x = msg.linear.x
    ang_z = msg.angular.z

def movebase_client():
    '''
    Sends coordinates to the move_base client to move the Turtlebot to that coordinate.
    '''
	# Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

	# Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

	# Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
	# Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
	# No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

	# Sends the goal to the action server.
    client.send_goal(goal)
	
	# Waits for the server to finish performing the action.
    #wait = client.wait_for_result()

	# If the result doesn't arrive, assume the Server is not available
    #if not wait:
        #rospy.logerr("Action server not available!")
        #rospy.signal_shutdown("Action server not available!")
    #else:
        # Result of executing the action
        #rospy.logerr("Action successful")
        #return client.get_result()


if __name__ == '__main__':
    '''
    The node waits for /odom message and x, y are assigned to the initial pose message.
    Node subscribes to the /cmd_vel topic.
    Checks for 10 consecutive instances of velocity of 0 before sending the Turtlebot back to the starting location.
    '''
    rospy.init_node('movebase_client_py')
    odom_initial_pose = rospy.wait_for_message("/odom", Odometry)
    rospy.Subscriber('/cmd_vel', Twist, vel_callback)

    # coords for the start position
    x = odom_initial_pose.pose.pose.position.x
    y = odom_initial_pose.pose.pose.position.y
    #print("The start position is:", x, y)

    cmd = Twist()
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0

    rate = rospy.Rate(10)
    
    counter = 0

    while not rospy.is_shutdown():			
        if lin_x == 0.0 and ang_z == 0.0:
            counter += 1
            if counter == num_of_instances:
                print("RUNNING THE CLIENT LOL")
                movebase_client()
                break
            rate.sleep()
        else:
            counter = 0
            rate.sleep()

    rospy.spin()