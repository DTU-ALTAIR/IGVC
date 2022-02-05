
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import cv2
import matplotlib.pyplot as plt
import message_filters
import numpy as np 
import sensor_msgs.msg
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class LaneFollower:

    def __init__(self):

       
        image_left = message_filters.Subscriber("/usb_cam_left/image_raw_left",Image)
        image_right = message_filters.Subscriber("/usb_cam_right/image_raw_right",Image)
        self.bridge_object = CvBridge()

        self.ts = message_filters.ApproximateTimeSynchronizer([image_left, image_right], 10, 0.5,allow_headerless=True)
        
        self.ts.registerCallback(self.camera_callback)
         

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
 
        self.move = Twist()

    

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
                if M['m00'] != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    #cv2.drawContours(image_frame, [c], -1, (0, 255, 0), 2)
                    cv2.circle(binary_image_mask, (cx, cy), 7, (0, 0, 255), -1)
                    print(flag + str(cx)+str(cy))
                    if cy>=400:
                        #self.move.linear.x = 0
                        print(flag +":" + "STOP" )
                    else:
                        #self.move.linear.x = 0.3
                        print(flag +":"+ "OK")
                    cv2.putText(binary_image_mask, "center", (cx - 20, cy - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 1000, (0, 0, 0), 2)
                    cv2.drawContours(binary_image_mask, [c], -1, (255,255,255), 2)
                #cv2.drawContours(blank_grid, [c], -1, (), -1)

        return black_image,blank_grid,binary_image_mask

    def camera_callback(self,image_left,image_right):
        try:

            cv_image = self.bridge_object.imgmsg_to_cv2(image_left,desired_encoding='bgr8')   
            cv_image_right = self.bridge_object.imgmsg_to_cv2(image_right,desired_encoding='bgr8')   
            

            cv_image_array = np.array(cv_image)
      
            cv_image_array_ = np.array(cv_image_right)
            #cv_image=np.uint8(cv_image)
            image = cv2.resize(cv_image_array,(700,700))
            image_right = cv2.resize(cv_image_array_,(700,700))

            lane = [(600,400),(675,550),(50,550),(200, 400)]

            #image_right = cv2.rotate(image_right, cv2.ROTATE_90_CLOCKWISE)

            black_image,blank_grid,binary_image_mask = self.detect_lane_in_a_frame(image,"Left")
            black_image_,blank_grid_,binary_image_mask_ = self.detect_lane_in_a_frame(image_right,"Right")

            self.pub.publish(self.move)
            #warp_image = cv2.resize(warp_image ,(700,700))
            #warp_image = cv2.cvtColor(warp_image,cv2.COLOR_BGR2GRAY)
            #warp_image = cv2.GaussianBlur(warp_image,(5,5),cv2.BORDER_DEFAULT)

            #warp_threshold = cv2.adaptiveThreshold(warp_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,25,7)

            
            plt.imshow(binary_image_mask_)
            #cv2.imshow("frame2",binary_image_mask_)
            plt.show()

            #cv2.imshow("frame1",warp_image)
            
            #cv2.imshow("depth",cv_image_norm)
       
            cv2.waitKey(0)

        except Exception as e:
            print(e)



def main():

    
    lane_follower_object = LaneFollower()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("SHUTTING DOWN ")

if __name__ =="__main__":

    rospy.init_node('lane_following_node',anonymous=True)


    main()