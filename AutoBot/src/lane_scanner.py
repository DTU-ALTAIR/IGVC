#!/usr/bin/env python
import rospy
import sys
import cv2
from scipy.spatial import distance as dist
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

PI=3.1415926535897
INCREMENT=0.01745329251
bridge = CvBridge()

lidx=400
lidy=400
class img_to_laser():
     def __init__(self):
        self.robot_frame          = rospy.get_param('~robot_frame', 'base_link')       
        self.map_frame            = rospy.get_param('~map_frame', 'gps')
        self.img_to_laser_sub = rospy.Subscriber("/usb_cam_center/image_raw_center", Image, self.range_finder)
        self.laser_to_laser_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.img_to_laser_pub = rospy.Publisher("/lane_scan_test", LaserScan, queue_size=1)
        self.ctrl_c = False
        self.rate = rospy.Rate(1) # 1hz
        self.scan=LaserScan()
        self.scan.ranges[0,180]
        self.scan.intensities[0,180]
        self.scan.header.frame_id=self.map_frame
        self.scan.angle_min = -3.14 + INCREMENT
        self.scan.angle_max = 3.14
        self.scan.angle_increment = INCREMENT
        self.scan.time_increment = 0
        self.scan.scan_time = 0.1
        self.scan.range_min = 0
        rospy.on_shutdown(self.shutdownhook)

     def scan_callback(self, msg):
         #print len(msg.ranges)
         for i in range(0,179):
            self.scan.ranges[i]= float('inf')
            self.scan.intensities[i]=0

     def range_finder(self, msg):
         #print len(msg.ranges)
         for i in range(0,179):
            self.scan.ranges[i]= float('inf')
            self.scan.intensities[i]=0

     def shutdownhook(self):

        self.ctrl_c = True

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
    lidar_obj=img_to_laser()
    lane_scanner=lidar_obj.range_finder(image)
    
def main(self):
    rospy.init_node('lane_scan', anonymous=True)
    image_topic="/usb_cam_center/image_raw_center"
    image_sub1 = rospy.Subscriber(image_topic, Image, image_callback)
    try:
        rospy.Rate(10)
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()




if __name__ == '__main__':
    main(sys.argv[1:])