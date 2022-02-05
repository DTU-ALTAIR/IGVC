#!/usr/bin/env python3
from cmath import cos, inf
from dis import dis
from mmap import MAP_PRIVATE

import sys
import cv2

import numpy as np


import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image



from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist

bridge = CvBridge()
move = Twist()



def laser_callback(laser_data):
    global move
    #print(laser_data.ranges)
    idx = [i/2 for i, arr in enumerate(laser_data.ranges) if np.isfinite(arr).all()]
    # distance = (2.596-(np.cos(96) * 2.64))/2
    print(laser_data.ranges)
    print(idx)
    # print(steering_angle)
   # print(idx)
    


def main(args):
  #print("test1")

  rospy.init_node('GridMapping', anonymous=True)

  image_topic="/usb_cam_right/image_raw_right"
  laser_scan_topic = "/scan"

  laser_scan = rospy.Subscriber(laser_scan_topic, LaserScan, laser_callback)
  laser_scan = rospy.Subscriber(laser_scan_topic, LaserScan, laser_callback)
  

  
  try:
    rospy.Rate(10)
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv[1:])