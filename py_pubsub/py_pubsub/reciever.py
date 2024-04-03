import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class Receiver(Node):


    def __init__(self):
        super().__init__("Receiver")
        self.create_subscription(String, '/msgs', self.callback, 1)

    def callback(self, msg):
        print("message received: %s" % msg)



def main(args = None):
    rclpy.init()

    try:
        #creates node
        node = Receiver()
        
        rclpy.spin(node)#runs until ctrl-c
        node.destroy_node()#destroy once stopped
        rclpy.shutdown()

    except KeyboardInterrupt:
        print('Node interrupted')

    finally:
        print("Node terminated")

    Receiver = Receiver()

    rclpy.spin(Receiver)

    Receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()