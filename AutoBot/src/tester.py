#!/usr/bin/env python3
from __future__ import print_function
import sys
import cv2
from scipy.spatial import distance as dist
import numpy as np
from rosgraph import rosenv
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
import tf
from math import pi
import matplotlib.pyplot as plt
bridge = CvBridge()

def order_points(pts):
    # sort the points based on their x-coordinates
    xSorted = pts[np.argsort(pts[:, 0]), :]

    # grab the left-most and right-most points from the sorted
    # x-roodinate points
    leftMost = xSorted[:2, :]
    rightMost = xSorted[2:, :]

    # now, sort the left-most coordinates according to their
    # y-coordinates so we can grab the top-left and bottom-left
    # points, respectively
    leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
    (tl, bl) = leftMost

    # now that we have the top-left coordinate, use it as an
    # anchor to calculate the Euclidean distance between the
    # top-left and right-most points; by the Pythagorean
    # theorem, the point with the largest distance will be
    # our bottom-right point
    D = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0]
    (br, tr) = rightMost[np.argsort(D)[::-1], :]

    # return the coordinates in top-left, top-right,
    # bottom-right, and bottom-left order
    return np.array([tl, tr, br, bl], dtype="float32")


def fpt(image, pts):
    # obtain a consistent order of the points and unpack them
    # individually
    rect = order_points(pts)
    (tl, tr, br, bl) = rect

    # compute the width of the new image, which will be the
    # maximum distance between bottom-right and bottom-left
    # x-coordiates or the top-right and top-left x-coordinates
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))

    # compute the height of the new image, which will be the
    # maximum distance between the top-right and bottom-right
    # y-coordinates or the top-left and bottom-left y-coordinates
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))

    # now that we have the dimensions of the new image, construct
    # the set of destination points to obtain a "birds eye view",
    # (i.e. top-down view) of the image, again specifying points
    # in the top-left, top-right, bottom-right, and bottom-left
    # order
    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype="float32")

    # compute the perspective transform matrix and then apply it
    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
    cv2.imshow("Warped Image window", warped)
    # return the warped image
    return warped

def publishPC2(img):
    img= cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    M= img.shape[0]
    #print(M)
    N= img.shape[1]
    #print(N)
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
             PointField('y', 4, PointField.FLOAT32, 1),
             PointField('z', 8, PointField.FLOAT32, 1),
             PointField('intensity', 12, PointField.FLOAT32, 1)]
    header = Header()
    header.frame_id = "base_link"
    pub = rospy.Publisher('/camera/depth/fakepoints', PointCloud2, queue_size=100)
    #point_test=point_cloud2()
    #print(img.shape)
    #print("HSDbvjfbsdkjfsdnfjkhs sdjf sdj")
    points_test=[]
    for x in range(1,M-1):
        for y in range(1,N-1):
            if (img[x][y]>200):
                header.stamp = rospy.Time.now()
                
                points = np.array([(189-x)/26,(85-y)/27,0,255]).T
                #pointtest.append(points)
                #np.append(points_test,points)
                points_test.append(points)    
    pc2 = point_cloud2.create_cloud(header, fields, points_test)
    pub.publish(pc2)
    
def read_rgb_image(image_name):
    rgb_image = image_name
    #rgb_image = cv2.imread(image_name)
    width = np.size(rgb_image,0)
    height = np.size(rgb_image,1)
    print([width,height]) #[512, 640]
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

    warped1 = fpt(image, pts)

    for (x, y) in pts:
       cv2.circle(image, (x, y), 5, (0, 255, 0), -1)

    warped1 = cv2.resize(warped1, (150, 175))
    filt_warped,_ = detect_lane_in_a_frame(warped1)

    #grid_obj = Gridmap()
    publishPC2(filt_warped)
    
    #map_publisher = grid_obj.publish_occupancygrid(filt_warped)


    #image publisher

    #cv2.imshow("Warped Image window", warped)
    #cv2.imshow("Masked Warped Image window", filt_warped)
    #cv2.imshow("Image window", image)
    plt.imshow(image)
    plt.show()
    cv2.waitKey(0)

def main(args):
  #print("test1")
  rospy.init_node('Imagetolaser', anonymous=True)
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