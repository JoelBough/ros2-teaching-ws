import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        self.create_subscription(LaserScan, '/scan', self.callback, 1)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        
    def callback(self, msg):
        cmd_vel = Twist()
        print(msg.ranges[int(len(msg.ranges)/2)])
        if(msg.ranges[int(len(msg.ranges)/2)] > 1.0):
            cmd_vel.linear.x = 0.2
        else:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.3
        self.cmd_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init()
    node = Robot()
    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('interrupted')
    finally:
        print('terminated')

if __name__ == '__main__':
    main()