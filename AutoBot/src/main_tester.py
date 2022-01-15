#!/usr/bin/env python3
from __future__ import print_function
import sys
import cv2
from scipy.spatial import distance as dist
import numpy as np
from invers_persp import four_point_transform
from rosgraph import rosenv
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import tf
from math import pi
import math

PI=3.1415926535897
INCREMENT=0.01745329251
NUM_READINGS=360
SCALING_FACTOR=100 

lid_x=0
lid_y=0

bridge = CvBridge()

class img_laser:
    
    def __init__(self):
        #rospy.init_node('RosGridMapping', anonymous=True)
        self.robot_frame          = rospy.get_param('~robot_frame', 'base_link')       
        self.map_frame            = rospy.get_param('~map_frame', 'gps')
        # Creata a OccupancyGrid message template
        self.map_msg = LaserScan()
        self.map_msg.header.frame_id = self.map_frame
        self.map_msg.angle_min = -PI + INCREMENT
        self.map_msg.angle_max = PI
        self.map_msg.angle_increment = INCREMENT
        self.map_msg.time_increment = 0
        self.map_msg.scan_time = 0.1
        self.map_msg.range_min = 0
        
        np.resize(np.array(self.map_msg.ranges),180)
        self.map_msg.intensities 

        #self.map_msg.info.origin.orientation.w = self.map_orientation_w
        self.map_pub = rospy.Publisher('lane_scan',LaserScan, queue_size=1)
        self.tf_sub = tf.TransformListener()
    
    def range_finder(self,image):
        
        for i in range(0,180):
            self.map_msg.ranges[i]=float('inf')
            self.map_msg.intensities[i]=0;
        try:
            current_time = rospy.Time.now()
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            cv2.imshow()
            self.map_msg.range_max = math.sqrt(image.cols * image.cols + image.rows * image.rows)
            for x in range(0,image.cols):
                for y in range(0,image.rows):
                    if(lid_x!=y & lid_y!=x):
                        x_dist=x-lid_x
                        y_dist=-(y-lid_y)
                        tot_dist=math.sqrt((x_dist*x_dist)+(y_dist*y_dist))
                        theta=math.floor(math.atan2(y_dist,x_dist))
                        theta=theta%180
                        if(tot_dist < self.map_msg.ranges[theta]):
                            self.map_msg.ranges[theta]=tot_dist
                            self.map_msg.intensities[theta]=10
            self.map_pub.publish(self.map_msg)
        except KeyboardInterrupt:
            print("Shutting down")

def read_rgb_image(image_name):
    rgb_image = image_name
    #rgb_image = cv2.imread(image_name)
    width = np.size(rgb_image,0)
    height = np.size(rgb_image,1)
    #print([width,height]) #[512, 640]
    #cv2.imshow("RGB Image",rgb_image)
    return rgb_image
        
def image_callback(ros_image):
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)    
    image = read_rgb_image(cv_image)
    lidar_obj=img_laser()
    lane_scanner=lidar_obj.range_finder(image)

def main(args):
  #print("test1")
  rospy.init_node('Image_to_laser', anonymous=True)
  image_topic="/usb_cam_center/image_raw_center"
  image_sub1 = rospy.Subscriber(image_topic, Image, image_callback)
  
  try:
    rospy.Rate(10)
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv[1:])