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
import time
import roslib
#import my_yolo as yolo

bridge = CvBridge()

INPUT_FILE='data/dog.jpg'
OUTPUT_FILE='predicted.jpg'
LABELS_FILE='data/coco.names'
CONFIG_FILE='cfg/yolov3.cfg'
WEIGHTS_FILE='yolov3.weights'
CONFIDENCE_THRESHOLD=0.3

LABELS = open(LABELS_FILE).read().strip().split("\n")

np.random.seed(4)
COLORS = np.random.randint(0, 255, size=(len(LABELS), 3),
	dtype="uint8")


net = cv2.dnn.readNetFromDarknet(CONFIG_FILE, WEIGHTS_FILE)



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
    pub = rospy.Publisher('/camera/depth/fakepoints', PointCloud2, queue_size=1)
    #point_test=point_cloud2()
    #print(img.shape)
    #print("HSDbvjfbsdkjfsdnfjkhs sdjf sdj")
    points_test=[]
    for y in range(1,N-1):
        for x in range(1,M-1):
            if (img[x][y]>200):
                header.stamp = rospy.Time.now()
                points = np.array([(190-x)/37,(76.5-y)/17.5,0,255]).T
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
def warp_perspective(lane,image):
        height = image.shape[0]
        width = image.shape[1]
        pts1 = np.float32([lane[3],lane[0],lane[2],lane[1]])
        pts2 = np.float32([[0,0],[width,0],[0,height],[width,height]])

        M = cv2.getPerspectiveTransform(pts1,pts2)
        dst = cv2.warpPerspective(image,M,(width,height))
        return dst,M

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
    
    image1 = read_rgb_image(cv_image)
    #image wrap
    #pts = np.array([(238, 262), (0, 395), (640-238, 262), (640, 395)])
    lane = [(640,0),(640,334),(0,334),(0, 0)]
            # for i in lane:
            #     image = cv2.circle(image,i,4,(0,255,0),-1)
    #warped1,M = warp_perspective(lane,image)
    #warped1 = fpt(image, pts)
    warped1 = image1[0:334,0:640]
    
    image = warped1
    (H, W) = image.shape[:2]

    # determine only the *output* layer names that we need from YOLO
    ln = net.getLayerNames()
    ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]


    blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416),
        swapRB=True, crop=False)
    net.setInput(blob)
    start = time.time()
    layerOutputs = net.forward(ln)
    end = time.time()


    print("[INFO] YOLO took {:.6f} seconds".format(end - start))


    # initialize our lists of detected bounding boxes, confidences, and
    # class IDs, respectively
    boxes = []
    confidences = []
    classIDs = []

    # loop over each of the layer outputs
    for output in layerOutputs:
        # loop over each of the detections
        for detection in output:
            # extract the class ID and confidence (i.e., probability) of
            # the current object detection
            scores = detection[5:]
            classID = np.argmax(scores)
            confidence = scores[classID]

            # filter out weak predictions by ensuring the detected
            # probability is greater than the minimum probability
            if confidence > CONFIDENCE_THRESHOLD:
                # scale the bounding box coordinates back relative to the
                # size of the image, keeping in mind that YOLO actually
                # returns the center (x, y)-coordinates of the bounding
                # box followed by the boxes' width and height
                box = detection[0:4] * np.array([W, H, W, H])
                (centerX, centerY, width, height) = box.astype("int")

                # use the center (x, y)-coordinates to derive the top and
                # and left corner of the bounding box
                x = int(centerX - (width / 2))
                y = int(centerY - (height / 2))

                # update our list of bounding box coordinates, confidences,
                # and class IDs
                boxes.append([x, y, int(width), int(height)])
                confidences.append(float(confidence))
                classIDs.append(classID)

    # apply non-maxima suppression to suppress weak, overlapping bounding
    # boxes
    idxs = cv2.dnn.NMSBoxes(boxes, confidences, CONFIDENCE_THRESHOLD,
        CONFIDENCE_THRESHOLD)

    # ensure at least one detection exists
    if len(idxs) > 0:
        # loop over the indexes we are keeping
        for i in idxs.flatten():
            # extract the bounding box coordinates
            (x, y) = (boxes[i][0], boxes[i][1])
            (w, h) = (boxes[i][2], boxes[i][3])

            color = [int(c) for c in COLORS[classIDs[i]]]

            cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
            text = "{}: {:.4f}".format(LABELS[classIDs[i]], confidences[i])
            cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, color, 2)

    # show the output image
    warped1=image
    
    cv2.imshow("Image window", image)
    
    
    
    
    #for (x, y) in lane:
    #    cv2.circle(image, (x, y), 5, (0, 255, 0), -1)

    warped1 = cv2.resize(warped1, (150, 175))
    filt_warped,_ = detect_lane_in_a_frame(warped1)
    
    #grid_obj = Gridmap()
    publishPC2(filt_warped)
    
    
    #map_publisher = grid_obj.publish_occupancygrid(filt_warped)


    #image publisher

    cv2.imshow("Warped Image window", warped1)
    cv2.imshow("Masked Warped Image window", filt_warped)
    cv2.imshow("Image window", image1)
    cv2.waitKey(1)
    

def main(args):
  #print("test1")
  rospy.init_node('Imagetolaser', anonymous=True)
  #for turtlebot3 waffle
  #image_topic="/camera/rgb/image_raw/compressed"
  #for usb cam
  image_topic="/usb_cam_center/image_raw_center"
  #print("test2")
  rospy.Subscriber(image_topic, Image, image_callback)
  try:
    rospy.Rate(10)
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv[1:])