import rclpy
from rclpy.node import Node
from ur_msgs.srv import SetIO
import time

class GripperROS(Node):
    def __init__(self):
        super().__init__('gripper_ros')
        self.cli = self.create_client(SetIO, '/io_and_status_controller/set_io')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Service not available, waiting...')

    def set_gripper(self, fun, pin, state):
        req = SetIO.Request()
        req.fun = fun
        req.pin = pin
        req.state = state
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def open_and_close(self):
        self.get_logger().info('Closing gripper...')
        self.set_gripper(fun=1, pin=16, state=1.0)  # Close
        time.sleep(2)
        self.set_gripper(fun=1, pin=16, state=0.0)

        self.get_logger().info('Opening gripper...')
        self.set_gripper(fun=1, pin=17, state=1.0)  # Open
        time.sleep(2)
        self.set_gripper(fun=1, pin=17, state=0.0)

def main(args=None):
    rclpy.init(args=args)
    node = GripperROS()
    node.open_and_close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
