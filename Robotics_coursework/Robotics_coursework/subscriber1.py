import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Receiver(Node):
    def __init__(self):
        super().__init__('receiver')
        self.create_subscription(String, '/msgs', self.callback, 1)

    def callback(self, msg):
        print("Received %s" % msg)

def main(args=None):
    rclpy.init()
    try:
        node = Receiver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('interrupted')
    finally:
        print('terminated')

if __name__ == '__main__':
    main()