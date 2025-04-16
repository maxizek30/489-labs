import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from custom_interfaces.action import RobotGoal

# Any additional imports here

# Decide your node class name
class Demure_Node(Node):
    def __init__(self):

        # Change to have your node name
        super().__init__('Chaewon')

        self.chaewon_action_client = ActionClient(self,RobotGoal,"whatever")

        while not self.chaewon_action_client.wait_for_server(1.0):
            self.get_logger().warn("Waiting for server...")

    def send_goal(self, x, y, theta):
        goal = RobotGoal.Goal()
        goal.goal_x = x
        goal.goal_y = y
        goal.goal_theta = theta
        self.chaewon_action_client.send_goal_async(goal, feedback_callback=self.goal_feedback_callback).add_done_callback(self.goal_response_callback)

    # Process Goal Accept/Reject
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info("GGEZ: Goal Accepted")
            goal_handle.get_result_async().add_done_callback(
            self.goal_result_callback)
        else:
            self.get_logger().info("FF go next: Goal Rejected")

    # Process Goal Feedback
    def goal_feedback_callback(self, feedback_msg):
        x = feedback_msg.feedback.current_x
        y = feedback_msg.feedback.current_y
        angle = feedback_msg.feedback.current_theta
        distance = feedback_msg.feedback.distance_from_goal
        string = f"x: {x}, y:{y} angle:{str(angle)}, distance: {str(distance)}"
        self.get_logger().info("Got feedback: " + string)

    # Process Action Result
    def goal_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success!")
        else:
            self.get_logger().info("Action aborted or cancelled")
                           
def main(args=None):
    rclpy.init(args=args)

    # Change to be your node class name
    node = Demure_Node()

    x = float(input("send x: "))
    y = float(input("send y: "))
    theta = float(input("send theta: "))

    node.send_goal(x,y,theta)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()