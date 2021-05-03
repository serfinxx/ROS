#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import some other modules from within this package
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry

class object_detection(object):

    def __init__(self):
        rospy.init_node('object_detection')
        self.base_image_path = '/home/student/myrosdata/week6_images'
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()
        
        # Default speed
        self.robot_controller.set_move_cmd(0.0, 0.0)
        self.status = 1
        

        self.move_rate = '' # fast, slow or stop
        self.stop_counter = 0

        # Shutdown hook 
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)
        
        self.m00 = 0
        self.m00_min = 100000

        # Blue, Red, Green, Turquoise, Purple, Yellow hsv colour values:
        self.lower = [(115, 224, 100), (0, 185, 100), (25, 150, 100), (75, 150, 100), (145, 185, 100), (28, 180, 100)]
        self.upper = [(130, 255, 255), (10, 255, 255), (70, 255, 255), (100, 255, 255), (150, 250, 255), (32, 255, 255)]

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True
    
    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, channels = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        colours = ['Blue','Red','Green','Turquoise','Purple','Yellow']

        for i in range(4):
            if i == 0:
                mask = cv2.inRange(hsv_img, self.lower[i], self.upper[i])
            else:
                mask = mask + cv2.inRange(hsv_img, self.lower[i], self.upper[i])

        m = cv2.moments(mask)
            
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        
        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def rotate_180_degree(self, init_yaw=0.0):
        init_yaw = (init_yaw+360)%360
        while abs(((init_yaw+180)+360)%360-((self.robot_odom.yaw+360)%360)) > 1:
            self.robot_controller.set_move_cmd(0.0, 0.2)
            self.robot_controller.publish()
            print(self.robot_odom.yaw)
        
        self.robot_controller.stop()

        


    def main(self):
        while not self.ctrl_c:
            # Strats here:
            if self.status == 1:
                # get current degree
                current_yaw = self.robot_odom.yaw
                # rotate 180 degrees
                self.rotate_180_degree(init_yaw=current_yaw)
                self.status += 1
            else:
                self.robot_controller.stop()
            
            




            
            self.robot_controller.publish()
            self.rate.sleep()
        
            
if __name__ == '__main__':
    object_detection = object_detection()
    try:
        object_detection.main()
    except rospy.ROSInterruptException:
        pass