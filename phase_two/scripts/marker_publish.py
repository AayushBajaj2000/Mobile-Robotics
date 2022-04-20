#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header
from fiducial_msgs.msg import FiducialTransformArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
import tf2_ros
import tf2_geometry_msgs 
import sensor_msgs.point_cloud2 as pcl2 
from sensor_msgs.msg import PointCloud2 as pc2
import dynamic_reconfigure.client

published = [] 
pub_pc = rospy.Publisher("/laserPointCloud", pc2, queue_size=0)
newPublish = rospy.Time(0)
lastPublish = rospy.Time(0)
cloudPoints = []
pose_1 = Pose()
rate_global = 0
client = 0 
paramChange = False

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
    global newPublish, lastPublish, cloudPoints,pose_1,rate_global,paramChange,client
    transformArray = data.transforms
    # Publish it only once -> one reading, maybe we should increase these
    
    if len(transformArray) > 0:
        transform = transformArray[0]
        '''
        if paramChange == False:
            client = dynamic_reconfigure.client.Client("move_base/DWAPlannerROS", timeout=20, config_callback=None)
            params = { 'min_vel_x' : -0.09, 'max_vel_x' : 0.09 }
            client.update_configuration(params)
            paramChange = True
        '''
        if transform.fiducial_id not in published:
            m_pub = rospy.Publisher('visualization_marker',  Marker, queue_size=1)
            
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = "raspicam"
            marker.type = marker.SPHERE
            marker.action = marker.ADD

            marker.ns = "fiducial"
            marker.id =  transform.fiducial_id
        
        # If we want to do it according to the map which we might need for phase 4.
            marker.pose.position.x = transform.transform.translation.x
            marker.pose.position.y = transform.transform.translation.y - 0.2
            marker.pose.position.z = transform.transform.translation.z
            marker.pose.orientation.x = transform.transform.rotation.x
            marker.pose.orientation.y = transform.transform.rotation.y
            marker.pose.orientation.z = transform.transform.rotation.z
            marker.pose.orientation.w = transform.transform.rotation.w

            marker.scale.x = 0.20
            marker.scale.y = 0.20
            marker.scale.z = 0.20

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration()
            
            # 1Hz
            rate = rospy.Rate(10)

            while not rospy.is_shutdown():
                connections = m_pub.get_num_connections()
                if connections > 0:
                    m_pub.publish(marker)
                    print("Published marker")
                    published.append(marker.id)
                    break
                rate.sleep()

            my_pose = Pose()
            my_pose.position.x = transform.transform.translation.x 
            my_pose.position.y = transform.transform.translation.y
            my_pose.position.z = transform.transform.translation.z

            transformed_pose = transform_pose(my_pose, "raspicam", "map")
            pose_1 = transformed_pose
            offsetX = 0.1
            offsetY = 0.1

            if len(cloudPoints) > 0:
                cloudPoints.append([pose_1.position.x - offsetX, pose_1.position.y + offsetY, pose_1.position.z + 0.25])
                increment = 0.006
                for x in range(40):
                    for y in range(40):
                        cloudPoints.append([pose_1.position.x - offsetX + x*increment, pose_1.position.y + offsetY - y*increment, pose_1.position.z + 0.25])
            else:
                points = [[pose_1.position.x - offsetX, pose_1.position.y + offsetY, pose_1.position.z + 0.25]]
                increment = 0.006
                for x in range(40):
                    for y in range(40):
                        points.append([pose_1.position.x + x*increment - offsetX, pose_1.position.y - y*increment + offsetY, pose_1.position.z + 0.25])
                cloudPoints = points
        else:  
            '''         
            if paramChange == True:
                client = dynamic_reconfigure.client.Client("move_base/DWAPlannerROS", timeout=20, config_callback=None)
                params = { 'min_vel_x' : -0.12, 'max_vel_x' : 0.12 }
                client.update_configuration(params)
                paramChange = False       
            '''
            # header
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'map'
            scaled = pcl2.create_cloud_xyz32(header, cloudPoints)
            pub_pc.publish(scaled)
            rate_global.sleep()
    elif pose_1:
        # header
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        scaled = pcl2.create_cloud_xyz32(header, cloudPoints)
        pub_pc.publish(scaled)
        rate_global.sleep()
        
def main():
    global rate_global
    rospy.init_node('publish_markers', anonymous=True)
    rate_global = rospy.Rate(25)
    rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
