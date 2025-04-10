import rclpy
from rclpy.node import Node
from custom_interfaces.srv import GetSummary

# Any additional imports here

# Decide your node class name
class stateSummary(Node):
    def __init__(self):

        # Change to have your node name
        super().__init__('state_summary_client_node')
        self.summary_client = self.create_client(GetSummary, 'get_state_summary')

    def request_summary(self, get_summary):
        request_obj = GetSummary.Request()
        request_obj.get_summary = get_summary

        self.response = self.summary_client.call_async(request_obj)

def main(args=None):
    rclpy.init(args=args)

    # Change to be your node class name
    node = stateSummary()
    yesorno = input('Do you want a summary? (yes/no): ')
    node.request_summary(yesorno)

    rclpy.spin_until_future_complete(node, node.response)
    summary_response = node.response.result()
    node.get_logger().info(str(summary_response.summary))
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()