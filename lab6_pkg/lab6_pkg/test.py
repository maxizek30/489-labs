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

import math

class MapPubNode(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        
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

        self.go_to_goal = ActionServer(self, RobotGoal,"go_to_goal",goal_callback=self.goal_callback,execute_callback=self.execute_callback)

        # Subscribe to the velocity unfiltered commands
        self.pos_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/robot1/pose', self.callback_pos, 10)
        self.pos_subscriber

        # Subscribe to the velocity unfiltered commands
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/robot1/map', self.callback_map, 10)
        self.map_subscriber

        # Subscirbe to scan
        self.scan_sub = self.create_subscription(LaserScan, '/robot1/scan', self.callback_scan, 10)
        self.scan_sub

        self.velocity_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)

    # Callback for pos sub
    def callback_pos(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation

        (_,_,self.ang) = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
       

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

    # Goal callback
    def goal_callback(self, goal_request):
        goal = [goal_request.goal_x,goal_request.goal_y]
        min_distance = 10000

        for obstacle in self.obstacle_space:
            distance = math.dist(goal,obstacle)
            if distance < min_distance:
                min_distance = distance

        if min_distance < self.robot_radius:
            self.get_logger().info("Rejected")
            return GoalResponse.REJECT
        
        self.get_logger().info("Accepted goal!")
        return GoalResponse.ACCEPT
    
    def execute_callback(self, goal_handle):
        goal_x = goal_handle.request.goal_x
        goal_y = goal_handle.request.goal_y
        goal_theta = goal_handle.request.goal_theta
        result = RobotGoal.Result()
        feedback = RobotGoal.Feedback()

        # What is close enough
        close_enough_distance = 0.25
        close_enough_angle = 0.2
        slow_threshold = 1.5

        # Base speeds
        rotation_speed = 0.25
        forward_speed = 0.4

        # Get initial distance
        distance_to_goal = math.dist([goal_x,goal_y],[self.x,self.y])

        # While not close enough
        while distance_to_goal > close_enough_distance:
            # New velocity msg
            vel = Twist()

            # Calc desired angle
            desired_angle = math.atan2(goal_y - self.y, goal_x - self.x)
            # Calc abs val difference between current angle and desired
            angle_dif = abs(self.ang - desired_angle)

            # If not close enough to desired angle
            if angle_dif > close_enough_angle:
                # Don't move forward
                vel.linear.x = 0.0
                
                # Rotate left
                if goal_y > self.y:
                    vel.angular.z = -rotation_speed
                # Rotate right
                else:
                    vel.angular.z = rotation_speed

            # If obstacle seen, stop
            elif self.obstacle:
                self.get_logger().info("Obstacle!")
                vel.linear.x = 0.0
                vel.angular.z = 0.0
                # Set result.success to False
                result.success = False
                # Exit while loop
                goal_handle.succeed()
                return result
            
            else: 
                vel.angular.z = 0.0
                # If farther than slow threshold
                if distance_to_goal > slow_threshold:
                    # Move at base speed
                    vel.linear.x = forward_speed
                else: 
                    # Move at speed relative to dist from goal
                    vel.linear.x = forward_speed * (distance_to_goal/slow_threshold)
            # Publish velocity
            self.velocity_pub.publish(vel)

            # Calc new distance to goal
            distance_to_goal = math.dist([goal_x,goal_y],[self.x,self.y])

            # Publish feedback
            feedback.current_x = float(round(self.x,2))
            feedback.current_y = float(round(self.y,2))
            feedback.current_theta = float(round(self.ang,2))
            feedback.distance = float(round(distance_to_goal,2))
            goal_handle.publish_feedback(feedback)

        # Calc dif between current angle and goal angle
        goal_angle_dif = abs(self.ang - goal_theta)
        # While not close enough
        while goal_angle_dif > close_enough_angle:
            vel = Twist()
            vel.linear.x = 0.0
            # Rotate
            vel.angular.z = rotation_speed
            # Publish 
            self.velocity_pub.publish(vel)

            # Calc dif between current angle and goal angle
            goal_angle_dif = abs(self.ang - goal_theta)

            # Publish feedback
            feedback.current_x = float(round(self.x,2))
            feedback.current_y = float(round(self.y,2))
            feedback.current_theta = float(round(self.ang,2))
            feedback.distance= float(round(distance_to_goal,2))
            goal_handle.publish_feedback(feedback)

        # Stop moving
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        # Publish
        self.velocity_pub.publish(vel)

        # Set result success to true
        result.success = True
        # Set status to succeed
        goal_handle.succeed()
        # Return result
        return result


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