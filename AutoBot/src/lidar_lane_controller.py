from cmath import inf
from random import randint
import random
from tracemalloc import start
from turtle import distance
import cv2
import numpy as np 
import math
import sys
import rospy
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import message_filters
from sensor_msgs.msg import  LaserScan 


class Detector():
    def __init__(self) -> None:
        self.bridge_object = CvBridge()

        image_left = message_filters.Subscriber("/usb_cam_left/image_raw_left",Image)
        image_right = message_filters.Subscriber("/usb_cam_right/image_raw_right",Image)
        self.distance_ = []
        self.values = []
        self.inf = np.float(inf)
        
        self.ts = message_filters.ApproximateTimeSynchronizer([image_left, image_right], 10, 0.5,allow_headerless=True)
        self.ts.registerCallback(self.camera_callback)

        self.laser = rospy.Publisher("/revised_scan",LaserScan,queue_size=50)
        self.scann = LaserScan()


    def warp_perspective(self,lane,image):
        height = image.shape[0]
        width = image.shape[1]
        pts1 = np.float32([lane[3],lane[0],lane[2],lane[1]])
        pts2 = np.float32([[0,0],[width,0],[0,height],[width,height]])

        M = cv2.getPerspectiveTransform(pts1,pts2)
        dst = cv2.warpPerspective(image,M,(width,height))
        return dst,M
    
    def filter_color(self,rgb_image, lower_bound_color, upper_bound_color):

        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        #cv2.imshow("hsv image",rgb_image)

        #define a mask using the lower and upper bounds of the yellow color 
        mask = cv2.inRange(rgb_image, lower_bound_color, upper_bound_color)

        return mask
    def detect_lane_in_a_frame(self,image_frame,flag):
        yellowLower =(0, 0, 100)
        yellowUpper = (35, 255, 255)
        image_frame = cv2.GaussianBlur(image_frame,(5,5),10)

        #cv2.imshow("frame",image_frame)
        #rgb_image = read_rgb_image(image_frame)
        binary_image_mask = self.filter_color(image_frame, yellowLower, yellowUpper)
        #contours = getContours(binary_image_mask)

        black_image = np.zeros([image_frame.shape[0], image_frame.shape[1],3],'uint8')
        blank_grid = np.zeros([image_frame.shape[0], image_frame.shape[1],3],'uint8')
        binary_image_mask = binary_image_mask[220:600,:]
        binary_image_mask = cv2.resize(binary_image_mask,(700,700))
        

        if flag == 'Left':
            binary_image_mask = cv2.rotate(binary_image_mask, cv2.ROTATE_90_COUNTERCLOCKWISE)
        elif flag == 'Right':
            binary_image_mask = cv2.rotate(binary_image_mask, cv2.ROTATE_90_CLOCKWISE)

        contours, hierarchy = cv2.findContours(binary_image_mask, 
                                                cv2.RETR_EXTERNAL,
                                                cv2.CHAIN_APPROX_SIMPLE)

        #contours_arr = []
        for c in contours:
            area = cv2.contourArea(c)
            
            if (area>1000):
                #cx, cy = get_contour_center(c)
                #contours_arr.append(get_contour_center(c))
                #cv2.drawContours(rgb_image, [c], -1, (150,250,150), 1)
                M = cv2.moments(c)
                if flag == "Right":
                    
                    if M['m00'] != 0:
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        #cv2.drawContours(image_frame, [c], -1, (0, 255, 0), 2)
                        cv2.circle(binary_image_mask, (cx, cy), 7, (0, 0, 255), -1)
                        distance = cx*0.004
                        print(distance)
                        
                        # if distance <=1.5:
                        #     self.move.linear.x = 0.2
                        #     self.move.angular.z = 0.2
                        #     #current_angle = current_angle + self.move.angular.z

                        # elif distance>2.3:
                        #     self.move.linear.x = 0.2
                        #     self.move.angular.z = -0.2
                        #     #current_angle = current_angle + self.move.angular.z
                        # else:
                        #     self.move.linear.x = 0.4
                        #     if current_angle<0:
                        #         self.move.angular.z = 0.2
                        #         current_angle = current_angle + 0.2
                        #     if current_angle>0:
                        #         self.move.angular.z = -0.2
                        #         current_angle = current_angle - 0.2
                        #     else:
                        #         self.move.linear.x = 0.2
                        #         self.move.angular.z = 0
                            

                #cv2.drawContours(blank_grid, [c], -1, (), -1)

        return black_image,blank_grid,binary_image_mask
    def perp(self,a) :
        b = np.empty_like(a)
        b[0] = -a[1]
        b[1] = a[0]
        return b

    # line segment a given by endpoints a1, a2
    # line segment b given by endpoints b1, b2
    # return 
    def seg_intersect(self,a1,a2, b1,b2) :

            da = a2-a1
            db = b2-b1
            dp = a1-b1
            dap = self.perp(da)
            denom = np.dot( dap, db)
            num = np.dot( dap, dp )
            return (num / denom.astype(float))*db + b1


    def get_coords(self,x, y, angle, imwidth, imheight):

        length = 1000


        x =  int(np.round(x + length * np.cos(angle * np.pi / 180.0)))
        y =  int(np.round(y + length * np.sin(angle * np.pi / 180.0)))
        return x,y 


    def camera_callback(self,image_left,image_right):
        try:
    
            current_time = rospy.Time.now()

            self.scann.header.stamp = current_time
            self.scann.header.frame_id = 'lidar'
            self.scann.angle_min = -1.535889983177185

            self.scann.angle_max = 1.535889983177185

            self.scann.angle_increment = 0.008556489832699299

            self.scann.time_increment = 0.0

            self.scann.range_min = 0.05000000074505806
            self.scann.range_max = 30.0

            

            image_data_left = self.bridge_object.imgmsg_to_cv2(image_left,desired_encoding="bgr8")
            
            image_data = self.bridge_object.imgmsg_to_cv2(image_right,desired_encoding="bgr8")

            cv_image_array = np.array(image_data)
        
            #cv_image_array_ = np.array(cv_image_right)
            #cv_image=np.uint8(cv_image)
            image = cv2.resize(cv_image_array,(700,700))
            #image_right = cv2.resize(cv_image_array,(700,700))

            lane = [(600,400),(675,550),(50,550),(200, 400)]
            black_image,blank_grid,binary_image_mask = self.detect_lane_in_a_frame(image,"Right")

            cv2.imshow("frame1",image_data)
            cv2.waitKey(1)
        except Exception as e:
            print(e)

def main():
    lane_follower_object = Detector()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("SHUTTING DOWN ")

if __name__ == "__main__":
    rospy.init_node('lane_following_node',anonymous=True)


    main()

       
