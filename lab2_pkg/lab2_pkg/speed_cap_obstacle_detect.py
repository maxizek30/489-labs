import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # Use the correct message type (Twist)
from std_msgs.msg import Header  # For using headers in the lights messagefro
from irobot_create_msgs.msg import LightringLeds
from sensor_msgs.msg import LaserScan 

# Assuming LightringLeds is a valid class from your codebase or custom message
# from your_custom_msgs.msg import LightringLeds  # Uncomment and use if it's a custom message

class SpeedCapObstacleDetect(Node):
    def __init__(self):
        # Initialize the node with the name 'speed_cap'
        super().__init__('speed_cap_obstacle_detect')
        
        # Create a subscription to 'cmd_vel_unfiltered' topic with Twist message type
        self.speed_cap_subscriber = self.create_subscription(Twist, '/robot1/cmd_vel_unfiltered', self.speed_cap_callback,  10)
        self.obstacle_detect_subscriber = self.create_subscription(LaserScan, '/robot1/scan', self.obstacle_callback, 10)
        self.speed_cap_publisher = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.lightring_publisher = self.create_publisher(LightringLeds, '/robot1/cmd_lightring', 10)
        self.obstacle_detected = False

    def speed_cap_callback(self, msg):
        # Assuming LightringLeds is a custom message type and initialized properly
        lights = LightringLeds()  # Replace with actual message
        lights.header.stamp = self.get_clock().now().to_msg()
        lights.override_system = True

        # Get the linear velocity from the Twist message (only considering x direction)
        speedometer = msg.linear.x
        speed_msg = Twist()
        speed_msg.linear.x = speedometer
        speed_msg.angular.z = msg.angular.z
        self.get_logger().info(f"Received speed: {speedometer}")
        

        # Cap speed at 0.4 and adjust the lights accordingly
        if speedometer > 0.4:
            speed_msg.linear.x = 0.4
            for i in range(6):  # Assuming we have 6 LEDs to controlZ
                lights.leds[i].red = 255
                lights.leds[i].green = 0
                lights.leds[i].blue = 255
        else:
            for i in range(6):  # Assuming we have 6 LEDs to control
                lights.leds[i].red = 255
                lights.leds[i].green = 255
                lights.leds[i].blue = 0
        if self.obstacle_detected:
            speed_msg.linear.x = 0.0
            for i in range(6):  # Assuming we have 6 LEDs to control
                lights.leds[i].red = 255
                lights.leds[i].green = 0
                lights.leds[i].blue = 0

        self.lightring_publisher.publish(lights)
        self.speed_cap_publisher.publish(speed_msg)

        # Log the speed
        self.get_logger().info(f"Speed after cap: {speedometer}")
    
    def obstacle_callback(self, msg):
        self.obstacle_detected = False
        min = msg.range_min
        max = msg.range_max
        laser_range = msg.ranges

        for i in range(200,340):
            if laser_range[i] > min and laser_range[i] < max:
                if laser_range[i] < 0.7:
                    self.obstacle_detected = True
                
        

def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the node
    node = SpeedCapObstacleDetect()

    # Keep the node spinning to process callbacks
    rclpy.spin(node)

    # Clean up and shut down when done
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


