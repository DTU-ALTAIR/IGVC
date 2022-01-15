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
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import tf
from math import pi
import math

PI=3.1415926535897
INCREMENT=0.01745329251
NUM_READINGS=360
SCALING_FACTOR=100 


bridge = CvBridge()

class img_laser:
    
    def __init__(self,lid_x,lid_y):
        #rospy.init_node('RosGridMapping', anonymous=True)
        self.robot_frame          = rospy.get_param('~robot_frame', 'base_link')       
        self.map_frame            = rospy.get_param('~map_frame', 'gps')
        self.map_resolution       = rospy.get_param('~map_resolution', 0.04)
        self.map_publish_freq     = rospy.get_param('~map_publish_freq', 12)
        # Creata a OccupancyGrid message template
        self.scan_ = LaserScan()
        self.scan_.header.frame_id = self.map_frame
        self.scan_.angle_min = -PI + INCREMENT
        self.scan_.angle_max = PI
        self.scan_.angle_increment = INCREMENT
        self.scan_.time_increment = 0
        self.scan_.scan_time = 0.1
        self.scan_.range_min = 0
        self.scan_.range_max = 1000000000
        self.lid_x=400
        self.lid_y=400
        np.resize(self.scan_.ranges,NUM_READINGS)   
        np.resize(self.scan_.intensities,NUM_READINGS)    
        #rospy.set_param("virtual_lidar_x", lid_x, 400)
        #rospy.set_param("virtual_lidar_y", lid_y, 400)
        #self.map_msg.info.origin.orientation.w = self.map_orientation_w
        self.scan_pub = rospy.Publisher('lane_scan',LaserScan, queue_size=1)
        self.tf_sub = tf.TransformListener()
    
    def range_finder(image):
            current_time = rospy.Time.now()
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            image.scan_.range_max= math.sqrt(image.cols * image.cols + image.rows * image.rows)
            for x in range(0,image.cols):
                for y in range(0,image.rows):
                    if(image.lid_x!=y & image.lid_y!=x):
                        x_dist=x-image.lid_x
                        y_dist=-(y-image.lid_y)
                        tot_dist=math.sqrt((x_dist*x_dist)+(y_dist*y_dist))
                        theta=math.floor(math.atan2(y_dist,x_dist))
                        theta=theta%360
                        if(tot_dist < image.map_msg.ranges[theta]):
                            image.map_msg.ranges[theta]=tot_dist
                            image.map_msg.intensities[theta]=10
            image.scan_pub=publish(image.scan_)
class Gridmap:
    def __init__(self):
        #rospy.init_node('RosGridMapping', anonymous=True)
        self.robot_frame          = rospy.get_param('~robot_frame', 'base_link')
        self.map_frame            = rospy.get_param('~map_frame', 'gps')
        self.map_center_x         = rospy.get_param('~map_center_x', 7.6)
        self.map_center_y         = rospy.get_param('~map_center_y', 3.50)
        self.map_orientation_x         = rospy.get_param('~map_orientation_x', -0.7071)
        self.map_orientation_y         = rospy.get_param('~map_orientation_y', 0.7071)
        self.map_orientation_z         = rospy.get_param('~map_orientation_z', 0)
        self.map_orientation_w         = rospy.get_param('~map_orientation_w', 0)
        # img size 640,700
        self.map_size_x           = rospy.get_param('~map_size_x', 6.0)
        self.map_size_y           = rospy.get_param('~map_size_y', 7.0)
        self.map_resolution       = rospy.get_param('~map_resolution', 0.04)
        self.map_publish_freq     = rospy.get_param('~map_publish_freq', 12)
       # self.update_movement      = rospy.get_param('~update_movement', 0.1)

        # Creata a OccupancyGrid message template
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = self.map_frame
        self.map_msg.info.resolution = self.map_resolution
        self.map_msg.info.width = int(self.map_size_x / self.map_resolution)
        self.map_msg.info.height = int(self.map_size_y / self.map_resolution)
        self.map_msg.info.origin.position.x = self.map_center_x
        self.map_msg.info.origin.position.y = self.map_center_y
        self.map_msg.info.origin.orientation.x = self.map_orientation_x
        self.map_msg.info.origin.orientation.y = self.map_orientation_y
        self.map_msg.info.origin.orientation.z = self.map_orientation_z
        #self.map_msg.info.origin.orientation.w = self.map_orientation_w

        self.map_pub = rospy.Publisher('lane_map', OccupancyGrid, queue_size=1)

        self.tf_sub = tf.TransformListener()
    
    def publish_occupancygrid(self, image):
        # Convert gridmap to ROS supported data type : int8[]
        # http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
        # The map data, in row-major order, starting with (0,0).  Occupancy probabilities are in the range [0,100].  Unknown is -1.
        
        # convert incoming image to map

        gridmap_p = self.img2grid(image)
        #unknown_mask = (gridmap_p == self.sensor_model_p_prior)  # for setting unknown cells to -1
        gridmap_int8 = (gridmap_p*100).astype(dtype=np.int8)
        #gridmap_int8[unknown_mask] = -1  # for setting unknown cells to -1

        # Publish map
        self.map_msg.data = gridmap_int8
        current_time = rospy.Time.now()
        #self.map_msg.header.stamp = stamp
        self.map_msg.header.stamp = current_time
        #self.tf_sub.waitForTransform(self.map_frame, self.robot_frame, self.map_msg.header.stamp, rospy.Duration(1.0))

        self.map_pub.publish(self.map_msg)
        rospy.loginfo_once("Published map!")

    def img2grid(self, image):
        img = (image/255)
        grid = img[:,:,0].flatten()
        return grid




def read_rgb_image(image_name):
    rgb_image = image_name
    #rgb_image = cv2.imread(image_name)
    width = np.size(rgb_image,0)
    height = np.size(rgb_image,1)
    #print([width,height]) #[512, 640]
    #cv2.imshow("RGB Image",rgb_image)

    return rgb_image

def filter_color(rgb_image, lower_bound_color, upper_bound_color):
    #convert the image into the HSV color space
    rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    #cv2.imshow("hsv image",rgb_image)

    #define a mask using the lower and upper bounds of the yellow color 
    mask = cv2.inRange(rgb_image, lower_bound_color, upper_bound_color)

    return mask

def detect_lane_in_a_frame(image_frame):
    yellowLower =(0, 0, 100)
    yellowUpper = (35, 255, 255)
    #rgb_image = read_rgb_image(image_frame)
    binary_image_mask = filter_color(image_frame, yellowLower, yellowUpper)
    #contours = getContours(binary_image_mask)

    black_image = np.zeros([image_frame.shape[0], image_frame.shape[1],3],'uint8')
    blank_grid = np.zeros([image_frame.shape[0], image_frame.shape[1],3],'uint8')
    contours, hierarchy = cv2.findContours(binary_image_mask.copy(), 
                                            cv2.RETR_EXTERNAL,
	                                        cv2.CHAIN_APPROX_SIMPLE)
    #contours_arr = []
    for c in contours:
        area = cv2.contourArea(c)
        if (area>150):
            #cx, cy = get_contour_center(c)
            #contours_arr.append(get_contour_center(c))
            #cv2.drawContours(rgb_image, [c], -1, (150,250,150), 1)
            cv2.drawContours(black_image, [c], -1, (255,255,255), -1)
            #cv2.drawContours(blank_grid, [c], -1, (), -1)

    return black_image,blank_grid

'''
def create_occupancy_grid(image):
    grid_msg = OccupancyGrid()
    grid_msg.data = image
    #grid_msg.info.
    return grid_msg
'''

def image_callback(ros_image):

    # define publisher
    #pub_lane_cent_left = rospy.Publisher('lane_cent_left', Float32MultiArray, queue_size=10)
    #print('got an image')
    global bridge
    #print(cx)
     
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    
    image = read_rgb_image(cv_image)
    #image wrap
    pts = np.array([(238, 262), (0, 395), (640-238, 262), (640, 395)])

    warped = four_point_transform(image, pts)

    for (x, y) in pts:
       cv2.circle(image, (x, y), 5, (0, 255, 0), -1)

    warped = cv2.resize(warped, (150, 175))
    #filt_warped,_ = detect_lane_in_a_frame(warped)

    #grid_obj = Gridmap()
    
    #map_publisher = grid_obj.publish_occupancygrid(filt_warped)
    lidar_obj=img_laser()
    lane_scanner=lidar_obj.range_finder(image)

    #image publisher

    cv2.imshow("Warped Image window", warped)
    #cv2.imshow("Masked Warped Image window", filt_warped)
    cv2.imshow("Image window", image)
    cv2.waitKey(1)

def main(args):
  #print("test1")
  rospy.init_node('GridMapping', anonymous=True)
  #for turtlebot3 waffle
  #image_topic="/camera/rgb/image_raw/compressed"
  #for usb cam
  image_topic="/usb_cam_center/image_raw_center"
  #print("test2")
  image_sub1 = rospy.Subscriber(image_topic, Image, image_callback)
  
  
  try:
    rospy.Rate(10)
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv[1:])