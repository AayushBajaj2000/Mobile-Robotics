#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header
from fiducial_msgs.msg import FiducialTransformArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf2_ros
import tf2_geometry_msgs 
from os import system

published = [] 
pose_1 = Pose()
rate_global = 0
client = 0 
paramChange = False

def movebase_client(x,y):
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
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
	# No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

	# Sends the goal to the action server.
    client.send_goal(goal)
    client.cancel_all_goals()


def transform_pose(input_pose, from_frame, to_frame):
   
    # **Assuming /tf2 topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time(0)

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1.5))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise

def callback(data):
    global published
    transformArray = data.transforms
    # Publish it only once -> one reading, maybe we should increase these
    
    if len(transformArray) > 0:
        odom_initial_pose = rospy.wait_for_message("/odom", Odometry)
        # coords for the start position
        odom_x = odom_initial_pose.pose.pose.position.x
        odom_y = odom_initial_pose.pose.pose.position.y
        movebase_client(odom_x, odom_y)
        
def main():
    rospy.init_node('publish_markers', anonymous=True)
    rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

