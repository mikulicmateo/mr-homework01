import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

class PositionSubscriber(Node):
    def __init__(self):
        super().__init__('position_subscriber')
        self.create_subscription(Odometry, "/odom", self.position_callback, 10)

    def position_callback(self, odom_msg):
        self.get_logger().info(f'Current position: \nx:{odom_msg.pose.pose.position.x}\ny:{odom_msg.pose.pose.position.y}')

def main(args = None):
    rclpy.init(args=args)

    position_subscriber = PositionSubscriber()
    rclpy.spin(position_subscriber)
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()