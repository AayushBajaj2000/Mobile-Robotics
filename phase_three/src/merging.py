#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

scan = LaserScan()

def callback(data):
    print(len(data.ranges))
    '''current_time = rospy.Time.now()
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
    sub = rospy.Subscriber('/laser_scan', LaserScan, callback)
    rospy.spin()