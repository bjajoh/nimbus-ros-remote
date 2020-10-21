#!/usr/bin/env python
import rospy
import time
import struct
import numpy as np

#ROS
import std_msgs.msg
from std_msgs.msg import String
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

#Nimbus
from nimbusPython import NimbusClient

def talker():
    pcl_pub = rospy.Publisher("/nimbus_ros/pointcloud", PointCloud2)

    fields =    [PointField('x', 0, PointField.FLOAT32, 1),
                 PointField('y', 4, PointField.FLOAT32, 1),
                 PointField('z', 8, PointField.FLOAT32, 1),
                 PointField('i', 12, PointField.FLOAT32, 1),
                ]

    rospy.init_node('nimbus_ros', anonymous=True)
    rospy.loginfo("Initializing nimbus-ros publisher node...")

    #connect to nimbus-sever
    cli = NimbusClient.NimbusClient(rospy.get_param('/nimbus_ip', '192.168.0.69'))
    time.sleep(0.5)

    #main loop
    while not rospy.is_shutdown():
        #get image data from nimbus
        header, (ampl, radial, x, y, z, conf) = cli.getImage(invalidAsNan=True)
        #convert pointcloud data for point_cloud2
        cloud = np.transpose(np.vstack((x.flatten(), y.flatten(), z.flatten(), ampl.flatten()))).tolist()
        header = Header()
        header.frame_id = "nimbus"
        pc2 = point_cloud2.create_cloud(header, fields, cloud)
        pc2.header.stamp = rospy.Time.now()
        pcl_pub.publish(pc2)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass