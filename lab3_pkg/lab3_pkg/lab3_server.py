import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # Use the correct message type (Twist)
from std_msgs.msg import Header  # For using headers in the lights messagefro
from irobot_create_msgs.msg import LightringLeds
from sensor_msgs.msg import LaserScan 
from sensor_msgs.msg import BatteryState
from irobot_create_msgs.msg import DockStatus
from custom_interfaces.srv import GetSummary

# Assuming LightringLeds is a valid class from your codebase or custom message
# from your_custom_msgs.msg import LightringLeds  # Uncomment and use if it's a custom message

class lab3_server(Node):
    def __init__(self):
        # Initialize the node with the name 'speed_cap'
        super().__init__('lab3_server')
        
        # Create a subscription to 'cmd_vel_unfiltered' topic with Twist message type
        self.speed_cap_subscriber = self.create_subscription(Twist, '/robot1/cmd_vel_unfiltered', self.speed_cap_callback,  10)
        self.obstacle_detect_subscriber = self.create_subscription(LaserScan, '/robot1/scan', self.obstacle_callback, 10)
        self.speed_cap_publisher = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.lightring_publisher = self.create_publisher(LightringLeds, '/robot1/cmd_lightring', 10)
        self.obstacle_detected = False

        # lab 3 stuff:
        self.speed_cap = 0.4
        self.battery_subscriber = self.create_subscription(BatteryState, '/robot1/battery_state', self.battery_callback, 10)
        self.charge_level = 0.0
        self.dock_subscriber = self.create_subscription(DockStatus, '/robot1/dock_status', self.dock_callback, 10)
        self.is_docked = False

        self.get_state_summary = self.create_service(GetSummary, 'get_state_summary', self.summary_callback)
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
        if speedometer > self.speed_cap:
            speed_msg.linear.x = self.speed_cap
            for i in range(6):  # Assuming we have 6 LEDs to control
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
        self.get_logger().info(f"Speed after cap: {speed_msg.linear.x}")
    
    def obstacle_callback(self, msg):
        self.obstacle_detected = False
        min = msg.range_min
        max = msg.range_max
        laser_range = msg.ranges

        for i in range(200,340):
            if laser_range[i] > min and laser_range[i] < max:
                if laser_range[i] < 0.7:
                    self.obstacle_detected = True

    def battery_callback(self, msg):
        self.charge_level = msg.percentage

    def dock_callback(self, msg):
        self.is_docked = msg.is_docked
    
    def summary_callback(self, request : GetSummary.Request, response : GetSummary.Response):
        summary = request.get_summary
        return_summary = ''

        if summary.lower() == 'yes':
            return_summary = f'Hello my name is: SwagBot69420_GGEZ, Battery Percentage: {self.charge_level}, Docked: {self.is_docked}, Current Speedcap: {self.speed_cap}'
        else:
            return_summary = 'NO SUMMARY FOR YOU THEN!'

        response.success = True
        response.summary = return_summary

        return response
        

def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the node
    node = lab3_server()

    # Keep the node spinning to process callbacks
    rclpy.spin(node)

    # Clean up and shut down when done
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()