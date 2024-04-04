import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Vision(Node):
    def __init__(self):
        super().__init__('vision')
        self.create_subscription(LaserScan, '/scan', self.callback, 1)
        self.publisher = self.create_publisher(String, '/timestamp', 1)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)


    def callback(self, msg):

        #print("Received: timestamp is %s" % msg.header.stamp.sec)
        #s = String()
        #s.data = str(msg.header.stamp.sec)
        #self.publisher.publish(s)

        cmd_vel = Twist()
        if msg.header.stamp.sec %2 == 0:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = -0.5
        else:
            cmd_vel.angular.z = 0.5
        self.cmd_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init()
    try:
        node = Vision()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('interrupted')
    finally:
        print('terminated')

if __name__ == '__main__':
    main()