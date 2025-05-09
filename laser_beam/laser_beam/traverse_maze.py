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
import numpy as np

import math

class MapPubNode(Node):
    def __init__(self):
        super().__init__('taehyung_node')
        
        self.PI = 3.14159265358979323846
        
        self.obstacle = False

        self.x = 0
        self.y = 0
        self.ang = 0

        self.robot_radius = 0.3

        self.obstacle_space = []

        self.resoltuion = 0.05 # Default 5cm
        self.width = 0
        self.height = 0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_ang = 0.0


        # Subscribe to the velocity unfiltered commands
        self.pos_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/robot1/pose', self.callback_pos, 10)
        self.pos_subscriber

        # Subscribe to the velocity unfiltered commands
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/robot1/map', self.callback_map, 10)
        self.map_subscriber

        # Subscirbe to scan
        self.scan_sub = self.create_subscription(LaserScan, '/robot1/scan', self.callback_scan, 10)

        # publish vel
        self.velocity_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)

    # Callback for pos sub
    def callback_pos(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation

        (_,_,self.ang) = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

        self.dx = round(math.cos(self.ang))
        self.dy = round(math.cos(self.ang))

    # index in map to real x y
    def index_to_real(self,col,row):
        real_x = round((col*self.resolution) + self.origin_x,2)
        real_y = round((row*self.resoltuion) + self.origin_y,2)
        return real_x,real_y

    # get occupancy grid and find free, unknown, obstacle space
    def callback_map(self,msg):
        self.resolution = round(msg.info.resolution,3)
        # The origin of the map [m, m, rad].  This is the real-world pose of the  cell (0,0) in the map
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.origin_ang = msg.info.origin.orientation.z
        
        self.width = msg.info.width
        self.height = msg.info.height

        robot_x = round((self.x-self.origin_x)/self.resolution)
        robot_y = round((self.y-self.origin_y)/self.resolution)
        # self.get_logger().info(str(robot_x) + ", " + str(robot_y) + ", " + str(self.ang*180/self.PI)) 

        robot_ang = round((self.ang - self.origin_ang)*180/self.PI,4)

        occupancy_grid = msg.data

        self.free_space = []
        self.unknown_space = []
        self.obstacle_space = []

        # occupancy grid (2d list)
        occupancy_grid_np = np.array(occupancy_grid)
        occupancy_grid_np.reshape(self.width, self.height)
        self.grid  = occupancy_grid_np
        
        row = 0
        while row < self.height:
            col = 0
            while col < self.width:
                real_x,real_y = self.index_to_real(col,row)
                point = occupancy_grid[col + (row*self.width)]
                if (col == robot_x and row == robot_y):
                    pass
                elif point == -1:
                    self.unknown_space.append([real_x,real_y]) 
                elif point == 0:
                    self.free_space.append([real_x,real_y])    
                else:
                    self.obstacle_space.append([real_x,real_y])         
                col += 1
            row += 1


    def callback_scan(self, msg):
        range_min = msg.range_min
        range_max = msg.range_max

        self.obstacle = False
        
        # front range is a:b (total range is 0:1080)
        a = 200
        b = 340

        for angle in range(a,b):
            scan_point = msg.ranges[angle]
            
            if (scan_point <= range_min) or (scan_point >= range_max):
                pass
            else:
                if scan_point <= 0.75:
                    # detect obstacle
                    self.obstacle = True
                    #self.get_logger().info("Obstacle detected!")

    def start(self):
        # while true
        while True:
            # if left clear, turn left
            if self.check_left():
                # turn left
                self.turn('left')
                return #TODO: implement
            # elif front clear, move forward
            elif self.check_front():
                # go
                return #TODO: implement
            # else turn right (maybe 180)
            elif self.check_right():
                # turn right
                self.turn('right')
                return #TODO: implement
        return

    def check_left(self):
        left_dx = -self.dy
        left_dy = self.dx

        gx, gy = self.real_to_index(self.x, self.y)

        left_gx = gx + left_dx
        left_gy = gy + left_dy

        return not self.is_occupied(left_gx, left_gy)

    def check_front(self):
        front_dx = self.dx
        front_dy = self.dy

        gx, gy = self.real_to_index(self.x, self.y)

        front_gx = gx + front_dx
        front_gy = gy + front_dy

        return not self.is_occupied(front_gx, front_gy)
    
    def check_right(self):
        right_dx = self.dy
        right_dy = -self.dx

        gx, gy = self.real_to_index(self.x, self.y)

        right_gx = gx + right_dx
        right_gy = gy + right_dy

        return not self.is_occupied(right_gx, right_gy)
    
    # real x and y to index in map
    def real_to_index(self,real_x, real_y):
        index_x = round((real_x-self.origin_x)/self.resolution)
        index_y = round((real_y-self.origin_y)/self.resolution)
        return index_x, index_y
    
    # check if obstacle
    def is_occupied(self, x, y):
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            return True
        return self.grid[y][x] >= 30

    def turn(self, type):
        if type == 'left':
            # turn left
            return
        elif type == 'right':
            # turn right
            return

def main(args=None): 

    try:
        rclpy.init(args=None)
        node = MapPubNode()

        # Use a MultiThreadedExecutor to enable processing goals concurrently
        executor = MultiThreadedExecutor()

        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
   
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()