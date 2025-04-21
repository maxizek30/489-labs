from irobot_create_msgs.msg import LightringLeds, AudioNote
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from custom_interfaces.action import RobotGoal

from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup
from tf_transformations import euler_from_quaternion

import numpy as np

import math

class GoToGoalNode(Node):
    def __init__(self):
        super().__init__('w_node')
        
        # pi
        self.PI = 3.14159265358979323846
        
        # False until scan sees something
        self.obstacle = False

        # Set by pose from pose topic
        self.x = 0
        self.y = 0
        self.ang = 0

        # Robot radius
        self.robot_radius = 0.3

        # Meta data from occupancy grid
        self.resoltuion = 0.05 # Default 5cm
        self.width = 0
        self.height = 0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_ang = 0.0

        # Subscribe to the robot position
        self.pos_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/robot1/pose', self.callback_pos, 10)
        self.pos_subscriber

        # Subscribe to the occupancy grid
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/robot1/map', self.callback_map, 10)
        self.map_subscriber

        # Subscirbe to LiDAR raw data
        self.scan_sub = self.create_subscription(LaserScan, '/robot1/scan', self.callback_scan, 10)
        self.scan_sub

        # Publisher for velocity
        self.velocity_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)

        # Action server
        self.action_server = ActionServer(self,RobotGoal,"whatever",goal_callback=self.goal_callback,execute_callback=self.execute_callback)

        self.obstacle_space = []

        # Map
        self.cost_map = None
        
        #List
        self.point_list = []

        # Occupancy grid
        self.occupancy_grid = None

    
    # HELPER FUNCTIONS

    def go_to_waypoint(self, waypoint, goal_handle, feedback):
        # waypoints should be real coordinates
        goal_x, goal_y = waypoint
        
        # Pythagorean Thrm
        distance = math.sqrt((((self.x - goal_x)**2) + ((self.y - goal_y)**2)))
        self.get_logger().info("Distance from goal: " + str(distance))

        while distance > 0.1:

            move = Twist()

            if self.obstacle == True:
                self.get_logger().info("Obstacle detected, cancelling goal")
                goal_handle.canceled()

            # angle we want
            angle = round((math.atan2((goal_y - self.y),(goal_x - self.x))), 1)

            if abs(angle - round(self.ang, 1)) > 0.2:
                move.angular.z = 0.2
                move.linear.x = 0.0

            else: 
                move.angular.z = 0.0
                move.linear.x = 0.3 * distance;
            
            #feedback fields
            feedback.current_x = float(self.x)
            feedback.current_y = float(self.y)
            feedback.current_theta = float(self.ang)
            feedback.distance_from_goal = float(distance)
             # Publish Feedback
            goal_handle.publish_feedback(feedback)

            #publish Twist message
            self.get_logger().info("publishing velocity. Linear x: " + str(move.linear.x) + " Angular z: " + str(move.angular.z) + "goal angle: " + str(angle))
            self.velocity_pub.publish(move)
           
            #calculate new distance
            distance = math.sqrt((((self.x - goal_handle.request.goal_x)**2) + ((self.y - goal_handle.request.goal_y)**2)))
        self.get_logger().info("arrived at waypoint, moving to next point")

     # Greedy Algorithm
    def greedy(self, x, y, goal_x, goal_y):
        goal_x_index, goal_y_index = self.real_to_index(goal_x, goal_y)
        # x and y should be indices
        x_index, y_index = self.real_to_index(x, y)
        self.grelper(x_index, y_index, goal_x_index, goal_y_index)

    # all attributes are indices
    def grelper(self, x, y, goal_x, goal_y, visited=None):
        if visited is None:
            visited = set()
        if (x, y) in visited:      # already looked at this cell
            return
        visited.add((x, y))
        self.point_list.append((x, y))

        if x == goal_x and y == goal_y:
            return

        cur_cost = self.cost_map[y][x]

        best = None
        best_cost = float('inf')
        for dx, dy in [(-1,-1), (-1,0), (-1,1),
                    ( 0,-1),         ( 0,1),
                    ( 1,-1), ( 1,0), ( 1,1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                c = self.cost_map[ny][nx]
                # **only move to a strictly lower cost**
                if c < best_cost and c < cur_cost:
                    best_cost, best = c, (nx, ny)

        if best is not None:                # found a downhill neighbour
            self.grelper(*best, goal_x, goal_y, visited)
        else:                               # stuck on a plateau — stop
            self.get_logger().warn(
                f"greedy search is stuck at ({x},{y}) with cost {cur_cost}"
            )


    # index in map to real x y
    def index_to_real(self,col,row):
        real_x = round((col*self.resolution) + self.origin_x,2)
        real_y = round((row*self.resoltuion) + self.origin_y,2)
        return real_x,real_y
    
    # real x and y to index in map
    def real_to_index(self,real_x, real_y):
        index_x = round((real_x-self.origin_x)/self.resolution)
        index_y = round((real_y-self.origin_y)/self.resolution)
        return index_x, index_y
       

    # CALLBACKS

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing execute_callback")
        

        result = RobotGoal.Result()
        feedback = RobotGoal.Feedback()
        goal_x = goal_handle.request.goal_x
        goal_y = goal_handle.request.goal_y
        goal_theta = goal_handle.request.goal_theta

        self.cost_map = np.full((self.height, self.width), float('inf'))

        goal_x_index, goal_y_index = self.real_to_index(goal_x, goal_y)
        
        # create cost_map
        for y in range(self.height):          # rows first
            for x in range(self.width):       # columns second
                dx = goal_x_index - x
                dy = goal_y_index - y
                self.cost_map[y][x] = math.hypot(dx, dy)

        for obstacle in self.obstacle_space:
            real_ox, real_oy = obstacle
            # translate obstacle position to map indices only once
            ox, oy = self.real_to_index(real_ox, real_oy)
            for y in range(max(0, oy-6), min(self.height, oy+7)):
                for x in range(max(0, ox-6), min(self.width, ox+7)):
                    if math.hypot(ox - x, oy - y) <= int(0.3 / self.resolution):
                        self.cost_map[y][x] = 1000



        self.cost_map[goal_y_index][goal_x_index] = 0
        
        self.get_logger().info(str(self.cost_map))
        # Pythagorean Thrm
        distance = math.sqrt((((self.x - goal_handle.request.goal_x)**2) + ((self.y - goal_handle.request.goal_y)**2)))
        self.get_logger().info("Distance from goal: " + str(distance))
        self.greedy(self.x, self.y, goal_x, goal_y)
        # go to each point
        for point in self.point_list:
            self.go_to_waypoint(point, goal_handle, feedback)

        #rotate to meet the desired goal_theta
        range_low = goal_theta - 0.3
        range_high = goal_theta + 0.3
        while(self.ang > range_high or self.ang < range_low):
            move = Twist()
            move.angular.z = 0.2
            move.linear.x = 0.0
            feedback.current_x = float(self.x)
            feedback.current_y = float(self.y)
            feedback.current_theta = float(self.ang)
            feedback.distance_from_goal = float(distance)
            goal_handle.publish_feedback(feedback)
             #publish Twist message
            self.get_logger().info("publishing velocity. Linear x: " + str(move.linear.x) + " Angular z: " + str(move.angular.z) + "goal angle: " + str(goal_theta))
            self.velocity_pub.publish(move)

        # Save result to result
        result.success = True

        # Set status to succeed
        goal_handle.succeed()
        return result


    def goal_callback(self, goal_request):
        self.get_logger().info("Received goal request")

        for obstacle in self.obstacle_space:
            self.get_logger().info("Obstacle")
            distance = math.sqrt((((obstacle[0] - goal_request.goal_x)**2) + ((obstacle[1] - goal_request.goal_y)**2)))
            
            if distance < 0.2:
                self.get_logger().info("There is an obstacle 0.3 m away, rejecting goal")
                return GoalResponse.REJECT
        self.get_logger().info("Goal accepted in server")
        return GoalResponse.ACCEPT
    
    def callback_pos(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.ang) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    # get occupancy grid and find free, unknown, obstacle space
    def callback_map(self,msg):
        self.resolution = round(msg.info.resolution,3)
        # The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.origin_ang = msg.info.origin.orientation.z
        
        # How many columns (width) and rows (height)
        self.width = msg.info.width
        self.height = msg.info.height

        # Map Setup
        # self.cost_map = [[0 for _ in range(self.width)] for _ in range(self.height)]

        # Where is robot in map frame in col,row indices (NOT METERS!!)
        robot_x = round((self.x-self.origin_x)/self.resolution)
        robot_y = round((self.y-self.origin_y)/self.resolution)
        
        # Occupancy grid data in one big list
        self.occupancy_grid = msg.data

        # Empty lists to add obstacles to
        self.obstacle_space = []
        
        # add all of the obstacles to a list
        row = 0
        while row < self.height:
            col = 0
            while col < self.width:
                real_x,real_y = self.index_to_real(col,row)
                point = self.occupancy_grid[col + (row*self.width)]

                if point > 25:
                    self.obstacle_space.append((real_x, real_y))

                col += 1
            row += 1

    def callback_scan(self, msg):
        # Check to make sure not any in front obstacles at any time
        ## NOTE: this is not our obstacle avoidance, this is still jsut stop if obstacle
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
                if scan_point <= 0.5:
                    # detect obstacle
                    self.obstacle = True

def main(args=None):
    try:
        rclpy.init(args=args)
        node = GoToGoalNode()
        # Use a MultiThreadedExecutor to enable processing goals concurrently
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()