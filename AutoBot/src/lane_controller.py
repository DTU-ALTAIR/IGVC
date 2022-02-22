#!/usr/bin/env python3
from cmath import cos, inf
from dis import dis
import sys
import cv2
from matplotlib.pyplot import sca
from scipy.spatial import distance as dist
import numpy as np
from sklearn.feature_extraction import image
from tomlkit import value
from torch import angle
from invers_persp import four_point_transform
from rosgraph import rosenv
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
import tf
from math import pi
from sensor_msgs.msg import LaserScan
import math

bridge = CvBridge()


def read_rgb_image(image_name):
    rgb_image = image_name
    #rgb_image = cv2.imread(image_name)
    width = np.size(rgb_image,0)
    height = np.size(rgb_image,1)
    #print([width,height]) #[512, 640]
    #cv2.imshow("RGB Image",rgb_image)

    return rgb_image
def perp(a) :
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b

# line segment a given by endpoints a1, a2
# line segment b given by endpoints b1, b2
# return 
def seg_intersect(a1,a2, b1,b2) :

        da = a2-a1
        db = b2-b1
        dp = a1-b1
        dap = perp(da)
        denom = np.dot( dap, db)
        num = np.dot( dap, dp )
        return (num / denom.astype(float))*db + b1


def get_coords(x, y, angle, imwidth, imheight):    

    length = 1000


    x =  int(np.round(x + length * np.cos(angle * np.pi / 180.0)))
    y =  int(np.round(y + length * np.sin(angle * np.pi / 180.0)))
    return x,y 

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
    kernel = np.ones((50,50),np.float32)/25

    #rgb_image = read_rgb_image(image_frame)
    binary_image_mask = filter_color(image_frame, yellowLower, yellowUpper)

    #contours = getContours(binary_image_mask)
    binary_image_mask = binary_image_mask[220:600,:]
    binary_image_mask = cv2.resize(binary_image_mask,(700,700))
    binary_image_mask = cv2.filter2D(binary_image_mask,-1,kernel)
    binary_image_mask = cv2.rotate(binary_image_mask, cv2.ROTATE_90_CLOCKWISE)
    
    black_image = np.zeros([image_frame.shape[0], image_frame.shape[1],3],'uint8')
    blank_grid = np.zeros([image_frame.shape[0], image_frame.shape[1],3],'uint8')
    contours, hierarchy = cv2.findContours(binary_image_mask.copy(), 
                                            cv2.RETR_EXTERNAL,
	                                        cv2.CHAIN_APPROX_SIMPLE)

                                        
    #contours_arr = []
    for c in contours:
        area = cv2.contourArea(c)
            
        if (area>1000):
            M = cv2.moments(c)
            area = cv2.contourArea(c)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                left = tuple(c[c[:, :, 0].argmin()][0])
                right = tuple(c[c[:, :, 0].argmax()][0])
                top = tuple(c[c[:, :, 1].argmin()][0])
                bottom = tuple(c[c[:, :, 1].argmax()][0])

                mid_point = ((top[0] + right[0])//2, (right[1] + left[1])//2)
                cv2.drawContours(image_frame, [c], -1, (0, 255, 0), 2)
                cv2.circle(binary_image_mask, (cx,cy), 10, (0, 0, 0), -1)

                #cv2.line(image_frame,(cx,int(endx)),(cy,int(endy)),(0,0,0),5,-1)
                distance = cx*0.003
             

                #cx, cy = get_contour_center(c)
                #contours_arr.append(get_contour_center(c))
                #cv2.drawContours(rgb_image, [c], -1, (150,250,150), 1)
                cv2.drawContours(black_image, [c], -1, (255,255,255), -1)
                #cv2.drawContours(blank_grid, [c], -1, (), -1)

    return black_image,blank_grid,binary_image_mask,image_frame,top,bottom,distance
'''
def create_occupancy_grid(image):
    grid_msg = OccupancyGrid()
    grid_msg.data = image
    #grid_msg.info.
    return grid_msg
'''

def image_callback(ros_image):


    inf_ = np.float(inf)
    # define publisher
    #pub_lane_cent_left = rospy.Publisher('lane_cent_left', Float32MultiArray, queue_size=10)
    #print('got an image')
    global bridge
    pc_pub = rospy.Publisher("/revised_scan",LaserScan,queue_size=50)

    current_time = rospy.Time.now()


    scann = LaserScan()
    scann.header.stamp = current_time
    scann.header.frame_id = 'lidar'
    scann.angle_min = -1.535889983177185

    scann.angle_max =0.785398 #1.535889983177185

    scann.angle_increment = 0.008556489832699299

    scann.time_increment = 0.0

    scann.range_min = 0.05000000074505806
    scann.range_max = 30.0

    reference_point = (0,350)
    #print(cx)
     
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    
    image = read_rgb_image(cv_image)
    image = cv2.resize(image,(700,700))
    
    #image wrap
    pts = [(600,400),(675,550),(50,550),(200, 400)]

   # warped,M = four_point_transform(image)

    #for (x, y) in pts:
    cv2.circle(image, reference_point, 5, (0, 255, 0), -1)

    #warped = cv2.resize(warped, (150, 175))
    filt_warped,_,binary_image_mask,image,top,bottom,value_distance = detect_lane_in_a_frame(image)

    #x,y = get_coords(0,350,340,700,700,)

    


    # x = int(x)
    # y = int(y)


    
    mid_point_x = (int(top[0])+int(bottom[0]))//2

    distance = [inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 2.22, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf]
    for i in range(0,60):
        x_,y_= get_coords(0,350,-i,700,700,)

        #mid_point_y = (int(top[1])+int(bottom[1]))//2
        #center_point = (mid_point_x,mid_point_y)
       
        point = seg_intersect(np.array((x_,y_)),np.array((1,350)),np.array([int(top[0]),int(top[1])]),np.array([int(bottom[0]),int(bottom[1])]))
        
        if point[1] >0:
            distance__ = point[0]*0.003
            value_distance_ = ((distance__)/math.cos(math.radians(i)))  
            
            distance[i] = value_distance_
            cv2.line(image,(0,350),(int(point[0]),int(point[1])),(0,255,0),3)
        else:
            
            distance[i] = inf


    scann.ranges = distance
    pc_pub.publish(scann)
    
    


    
    #map_publisher = grid_obj.publish_occupancygrid(filt_warped)


    #image publisher

    cv2.imshow("Warped Image window",image)
    cv2.imshow("Masked Warped Image window", binary_image_mask)
    #cv2.imshow("Image window", image)
    cv2.waitKey(1)

def main(args):
  #print("test1")
  rospy.init_node('GridMapping', anonymous=True)
  #for turtlebot3 waffle
  #image_topic="/camera/rgb/image_raw/compressed"
  #for usb cam
  image_topic="/usb_cam_right/image_raw_right"
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