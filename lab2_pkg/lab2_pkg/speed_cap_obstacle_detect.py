from turtle import up
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from irobot_create_msgs.msg import LightringLeds
from sensor_msgs.msg import LaserScan
import cv2
from cv_bridge import CvBridge as cvb
from sensor_msgs.msg import Image
from pyzbar.pyzbar import decode
import numpy as np

class teleop_demo(Node):
    def __init__(self):
        super().__init__('teleop_demo_node')
        self.red_detected = False
        
        self.speed_cap_subscriber = self.create_subscription(Twist, '/robot1/cmd_vel_unfiltered', self.stop_on_red,  10)
        self.obstacle_detect_subscriber = self.create_subscription(LaserScan, '/robot1/scan', self.obstacle_callback, 10)
        self.speed_cap_publisher = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.lightring_publisher = self.create_publisher(LightringLeds, '/robot1/cmd_lightring', 10)
        self.obstacle_detected = False

        # camera vision sub
        self.cam_sub = self.create_subscription(Image, 'robot1/oakd/rgb/preview/image_raw',self.cam_callback, 10)
        self.bridge = cvb()
        self.codes = []

        self.x = 0
        self.y = 0
        self.ang = 0
        
        self.pos_subscriber = self.create_subscription(
            PoseWithCovarianceStamped, '/robot1/pose', self.callback_pos, 10)



     def callback_pos(self, msg):
        self.get_logger().info("getting position")
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        (_, _, self.ang) = self.quaternion_to_euler(quaternion.x, quaternion.y, quaternion.z, quaternion.w)

    def stop_on_red(self, msg):
        lights = LightringLeds()
        lights.header.stamp = self.get_clock().now().to_msg()
        lights.override_system = True

        speedometer = msg.linear.x

        speed_msg = Twist()
        speed_msg.linear.x = speedometer
        speed_msg.angular.z = msg.angular.z
        
        if self.red_detected:
            speed_msg.linear.x = 0.0 # stop
            self.get_logger().info("Stopping...")
            for i in range(6):
                lights.leds[i].red = 255
                lights.leds[i].green = 0
                lights.leds[i].blue = 0

        else:
            for i in range(6):
                lights.leds[i].red = 0
                lights.leds[i].green = 255
                lights.leds[i].blue = 0

        self.lightring_publisher.publish(lights)
        self.speed_cap_publisher.publish(speed_msg)
    
    def obstacle_callback(self, msg):
        self.obstacle_detected = False
        min = msg.range_min
        max = msg.range_max
        laser_range = msg.ranges

        for i in range(200,340):
            if laser_range[i] > min and laser_range[i] < max:
                if laser_range[i] < 0.7:
                    self.obstacle_detected = True
                
    def cam_callback(self, msg):
        try:
            #stuff
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.waitKey(1)
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # ranges
            upper_range1 = (10,255,255)
            lower_range1 = (0,100,100)
            upper_range2 = (180,255,255)
            lower_range2 = (160,100,100)
            mask1 = cv2.inRange(hsv_image, lower_range1, upper_range1)
            mask2 = cv2.inRange(hsv_image, lower_range2, upper_range2)

            mask = cv2.bitwise_or(mask1, mask2)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            height, width = hsv_image.shape[:2]
            area_threshold = 0.20 * (height * width)

            flag = False
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > area_threshold:
                    flag = True
            if flag:
                self.red_detected = True
                self.get_logger().info("RED DETECTED")
            else: 
                self.red_detected = False

            barcodes = decode(cv_image)
            if barcodes:
                self.get_logger().info("QR CODE SEEN")
                for barcode in barcodes:
                    (x,y,w,h) = barcode.rect
                    data = barcode.data.decode("utf-8")
                    code_type = barcode.type

                    if data in self.codes:
                        continue
                    if w == 0:
                        continue
                    
                    distance = (0.174) / w

                    x_qr = self.x + np.cos(self.ang) * distance
                    y_qr = self.y + np.sin(self.ang) * distance

                    self.codes[data] = (x_qr, y_qr)

            cv2.imshow("Camera feed", cv_image)
        except Exception as e:
            self.get_logger().error(f"Failed to process: {e}")

def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the node
    node = teleop_demo()

    # Keep the node spinning to process callbacks
    rclpy.spin(node)

    # Clean up and shut down when done
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


