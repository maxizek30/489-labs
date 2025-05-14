from irobot_create_msgs.msg import LightringLeds, AudioNote
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from custom_interfaces.action import RobotGoal
from tf_transformations import euler_from_quaternion

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

        self.grid = []

        self.pos_subscriber = self.create_subscription(
            PoseWithCovarianceStamped, '/robot1/pose', self.callback_pos, 10, callback_group=self.callback_group)

        self.map_subscriber = self.create_subscription(
            OccupancyGrid, '/robot1/map', self.callback_map, 10, callback_group=self.callback_group)

        self.scan_sub = self.create_subscription(
            LaserScan, '/robot1/scan', self.callback_scan, 10, callback_group=self.callback_group)

        self.velocity_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)

        self.create_timer(1.0, self.start, callback_group=self.callback_group)

    def callback_pos(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        (_, _, self.ang) = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        self.dx = round(math.cos(self.ang))
        self.dy = round(math.sin(self.ang))

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
        
        if self.check_left():
            print("turning left")
                # self.turn('left')
            # elif self.check_front():
            #     print("moving forward")
            #     self.move_forward()
            # elif self.check_right():
            #     print("turning right")
            #     self.turn('right')
    


    def check_left(self):
        left_dx = -self.dy * 4
        left_dy = self.dx * 4
        gx, gy = self.real_to_index(self.x, self.y)
        return not self.is_occupied(gx + left_dx, gy + left_dy)

    def check_front(self):
        gx, gy = self.real_to_index(self.x, self.y)
        return not self.is_occupied(gx + self.dx, gy + self.dy)

    def check_right(self):
        right_dx = self.dy * 4
        right_dy = -self.dx * 4
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
