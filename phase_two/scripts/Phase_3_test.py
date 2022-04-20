#!/usr/bin/env python
import math
import rospy
from std_msgs.msg import String, Header
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2 as pc2
import sensor_msgs.point_cloud2 as pcl2 
from laser_geometry import LaserProjection


scan = LaserScan()
# pub = rospy.Publisher("/scan_laser", LaserScan, queue_size=10)
pub = rospy.Publisher("/laserPointCloud", pc2, queue_size=10)

def callback(data):
    # global scan
    #laserProj = LaserProjection()
    #cloud_out = laserProj.projectLaser(data)
    #pub.publish(cloud_out)
    
    cloudPoints = [[1,1,0],[1,0,0], [1,0.5,0]]
    # header
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'base_scan'
    scaled = pcl2.create_cloud_xyz32(header, cloudPoints)
    pub.publish(scaled)
    ''' current_time = rospy.Time.now()
    scan.header.stamp = current_time
    scan.header.frame_id = "base_scan"
    scan.angle_min = -3.14
    scan.angle_max = 3.14
    scan.angle_increment = 0
    scan.time_increment = 0.00004999999
    scan.range_min = 0
    scan.range_max = 100.0

    ranges = []
    xt = 0
    for x in range(72):
        increment = a + radius / 72.0
        y = b + math.sqrt(radius**2 - (xt - a)**2)
        xt += increment
        ranges.append(y)

    intensities = []
    for x in range(72):
        intensities.append(10)

    scan.ranges = ranges
    scan.intensities = intensities
    
    print(scan)
    pub.publish(scan)'''

if __name__ == '__main__':
    rospy.init_node('testing', anonymous=True)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()


