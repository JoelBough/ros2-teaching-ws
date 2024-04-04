import rclpy
from rclpy.node import Node
from std_msgs.msg import String 

class Chatter(Node):
    def __init__(self):
        super().__init__('chatter')
        self.publisher = self.create_publisher(String, '/msgs', 1)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.run_step)
        self.counter  = 0

    def run_step(self):
        data_object = String()
        data_object.data = 'counter: %d' % self.counter
        self.counter += 1
        print("publishing %s"%data_object)
        self.publisher.publish(data_object)

def main(args=None):
    rclpy.init()
    try:
        node = Chatter()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('interrupted')
    finally:
        print('terminated')
if __name__ == '__main__':
    main()
