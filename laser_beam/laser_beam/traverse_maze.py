from irobot_create_msgs.msg import LightringLeds, AudioNote
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from custom_interfaces.action import RobotGoal
import cv2
from cv_bridge import CvBridge as cvb
from sensor_msgs.msg import Image
from pyzbar.pyzbar import decode

from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import time
import numpy as np
import math
import os


class MapPubNode(Node):
    def __init__(self):
        super().__init__('taehyung_node')
        self.callback_group = ReentrantCallbackGroup()

        self.PI = math.pi
        self.obstacle = False

        self.x = 0
        self.y = 0
        self.ang = 0

        self.robot_radius = 0.3
        self.obstacle_space = []
        self.resolution = 0.05
        self.width = 0
        self.height = 0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_ang = 0.0
        self.dx = 0.0
        self.dy = 0.0
        self.red_detected = False

        self.grid = []

        self.pos_subscriber = self.create_subscription(
            PoseWithCovarianceStamped, '/robot1/pose', self.callback_pos, 10, callback_group=self.callback_group)

        self.map_subscriber = self.create_subscription(
            OccupancyGrid, '/robot1/map', self.callback_map, 10, callback_group=self.callback_group)

        self.scan_sub = self.create_subscription(
            LaserScan, '/robot1/scan', self.callback_scan, 10, callback_group=self.callback_group)

        self.velocity_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)

        self.create_timer(1.0, self.start, callback_group=self.callback_group)

        self.cam_sub = self.create_subscription(Image, "/robot1/oakd/rgb/preview/image_raw", self.cam_callback, 10)
        self.bridge = cvb()
        
    def callback_pos(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        (_, _, self.ang) = self.quaternion_to_euler(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        self.dx = round(math.cos(self.ang))
        self.dy = round(math.sin(self.ang))
    def cam_callback(self, msg):
        print("in cam callback")
        try:
            #stuff
            print("in cam try block")
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Camera feed", cv_image)
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
            else: 
                self.red_detected = False

            barcodes = decode(cv_image)
            if barcodes:
                for barcode in barcodes:
                    (x,y,w,h) = barcode.rect
                    data = barcode.data.decode("utf-8")
                    
                    if data in self.qr_codes:
                        continue
                    if w == 0:
                        continue

  

        except Exception as e:
            self.get_logger().error(f"Failed to process: {e}")
    def index_to_real(self, col, row):
        real_x = round((col * self.resolution) + self.origin_x, 2)
        real_y = round((row * self.resolution) + self.origin_y, 2)
        return real_x, real_y

    def callback_map(self, msg):
        self.resolution = round(msg.info.resolution, 3)
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.origin_ang = msg.info.origin.orientation.z

        self.width = msg.info.width
        self.height = msg.info.height
        occupancy_grid = msg.data

        self.free_space = []
        self.unknown_space = []
        self.obstacle_space = []

        occupancy_grid_np = np.array(occupancy_grid)
        self.grid = occupancy_grid_np.reshape((self.height, self.width))
        

        for row in range(self.height):
            for col in range(self.width):
                real_x, real_y = self.index_to_real(col, row)
                point = occupancy_grid[col + (row * self.width)]
                if point == -1:
                    self.unknown_space.append([real_x, real_y])
                elif point == 0:
                    self.free_space.append([real_x, real_y])
                else:
                    self.obstacle_space.append([real_x, real_y])

    def callback_scan(self, msg):
        range_min = msg.range_min
        range_max = msg.range_max
        self.obstacle = False
        for angle in range(200, 340):
            scan_point = msg.ranges[angle]
            if range_min < scan_point < range_max and scan_point <= 0.75:
                self.obstacle = True
                break

    def start(self):
        print("started")
        twist = Twist()
        while True:
            if self.check_left():
                print("left opengo")
                twist.linear.x = 0.0
            elif (self.check_front()):
                twist.linear.x = 0.4
                print("front open")
            elif self.check_right():
                twist.linear.x = 0.0
                print("right open")

            if self.red_detected:
                print("stopping red detected")
                twist.linear.x = 0.0
                
            self.velocity_pub.publish(twist)

                # self.turn('left')
            # elif self.check_front():
            #     print("moving forward")
            #     self.move_forward()
            # elif self.check_right():
            #     print("turning right")
            #     self.turn('right')
    


    def check_left(self):
        left_dx = -self.dy * 26
        left_dy = self.dx * 26
        gx, gy = self.real_to_index(self.x, self.y)
        return not self.is_occupied(gx + left_dx, gy + left_dy)

    def check_front(self):
        gx, gy = self.real_to_index(self.x, self.y)
        front_dx = self.dx * 12
        front_dy = self.dy * 12
        return not self.is_occupied(gx + front_dx, gy + front_dy)

    def check_right(self):
        right_dx = self.dy * 26
        right_dy = -self.dx * 26
        gx, gy = self.real_to_index(self.x, self.y)
        return not self.is_occupied(gx + right_dx, gy + right_dy)

    def real_to_index(self, real_x, real_y):
        return int((real_x - self.origin_x) / self.resolution), int((real_y - self.origin_y) / self.resolution)

    def is_occupied(self, x, y):
        x = int(x)
        y = int(y)
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            return True
        copy = self.grid
        copy[y][x] = -2
        gx, gy = self.real_to_index(self.x, self.y)
        copy[gy][gx] = -3

        self.save_grid_to_file(copy)
        print(f'x: {x} y: {y}')
        return self.grid[y][x] >= 30

    def normalize_angle(self, angle):
        while angle > self.PI:
            angle -= 2 * self.PI
        while angle < -self.PI:
            angle += 2 * self.PI
        return angle

    def angle_reached(self, current, target, direction):
        current = self.normalize_angle(current)
        target = self.normalize_angle(target)
        if direction == 'left':
            return current >= target if target > current else current >= target or current < -self.PI/2
        else:
            return current <= target if target < current else current <= target or current > self.PI/2

    def turn(self, type):
        twist = Twist()
        angular_speed = 0.5  # rad/s — tune this for your robot
        duration = self.PI / 2 / abs(angular_speed)  # Time to turn 90°

        if type == 'left':
            twist.angular.z = angular_speed
        elif type == 'right':
            twist.angular.z = -angular_speed
        else:
            return

        # Start rotating
        start_time = time.time()
        while time.time() - start_time < duration:
            self.velocity_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)

        # Stop
        twist.angular.z = 0.0
        self.velocity_pub.publish(twist)


    def move_forward(self, distance=0.3, speed=0.1):
        twist = Twist()
        twist.linear.x = speed
        duration = distance / speed
        start_time = time.time()
        while time.time() - start_time < duration:
            self.velocity_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)
        twist.linear.x = 0.0
        self.velocity_pub.publish(twist)
    
    # maxizek30
    def save_grid_to_file(self, map):
        desktop_path = os.path.join(os.path.expanduser("~"), "Desktop", "output.txt")
        # Map values to emojis: -1 (unknown): 🟦, 0 (free): ⬜, >=30 (occupied): 🟥, else: ⬛
        def cell_to_emoji(cell):
            if cell == -1:
                return "🟦"
            elif cell == 0:
                return "⬜"
            elif cell >= 30:
                return "🟥"
            elif cell == -2:
                return "🟩"
            elif cell == -3:
                return "🟫"
            else:
                return "⬛"

        with open(desktop_path, "w") as f:
            for row in map:
                f.write("".join(cell_to_emoji(cell) for cell in row) + "\n")

    def quaternion_to_euler(self, x, y, z, w):
        # Roll (x-axis rotation)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        # Pitch (y-axis rotation)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        # Yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw


def main(args=None):
    try:
        rclpy.init(args=args)
        node = MapPubNode()
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
