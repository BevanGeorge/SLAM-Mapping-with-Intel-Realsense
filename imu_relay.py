import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuRelay(Node):
    def __init__(self):
        super().__init__('imu_relay')
        self.sub = self.create_subscription(Imu, '/camera/camera/imu', self.callback, 10)
        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        self.get_logger().info("Relaying /camera/camera/imu -> /imu/data")

    def callback(self, msg):
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
