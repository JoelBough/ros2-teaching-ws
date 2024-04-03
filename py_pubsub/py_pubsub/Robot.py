import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Robot(Node):

    def __init__(self):
        #calls constructor of super class in node
        super().__init__('Robot')

        #creates publisher for type string and topic name /msgs length of queue 1(last message)
        self.publisher_ = self.create_publisher(String, '/msgs', 1)

        #creates timer triggering the run step callback
        timer_period = 1 #seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0
    
    def timer_callback(self):
        #main function that is run by timer
        data_object = String() #creates string object

        data_object.data = 'Hello World: %d' % self.counter
        self.publisher_publish(data_object)

def main(args = None):
    rclpy.init()

    try:
        #creates node
        node = Robot()
        rclpy.spin(node)#runs until ctrl-c
        node.destroy_node()#destroy once stopped
        rclpy.shutdown()

    except KeyboardInterrupt:
        print('Node interrupted')

    finally:
        print("Node terminated")

    Robot = Robot()

    rclpy.spin(Robot)

    Robot.destrpy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()