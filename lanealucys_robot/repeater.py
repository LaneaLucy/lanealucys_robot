import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header


class repeater(Node):

    def __init__(self):
        super().__init__('repeater')
        
        #self.message = PoseWithCovarianceStamped()
        
        self.declare_parameter('input', 'input')
        self.declare_parameter('output', 'output')
        self.declare_parameter('msgs_per_sec', 2.0)
        
        self.input_topic = self.get_parameter('input').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output').get_parameter_value().string_value
        self.msgs_per_sec = self.get_parameter('msgs_per_sec').get_parameter_value().double_value
        
        self.get_logger().info('Input Topic: "%s"' % self.input_topic)
        self.get_logger().info('Output Topic: "%s"' % self.output_topic)
        self.get_logger().info('msgs_per_sec: "%f"' % self.msgs_per_sec)
        
        #print('rclpy:')
        #print(rclpy.time.Time())
        #print('node:')
        #print(Node.get_clock(self).now())
        
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, self.output_topic, 10)
        timer_period = 1 / self.msgs_per_sec
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.input_topic,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.message = msg
        self.get_logger().info('I heard...')

    def timer_callback(self):
        #print('rclpy:')
        #print(rclpy.time.Time())
        #print('node:')
        #print(self.get_clock().now())
        if hasattr(self, 'message'):
            self.message.header.stamp = Node.get_clock(self).now().to_msg()
            self.publisher_.publish(self.message)
            self.get_logger().info('Publishing...')


def main(args=None):
    rclpy.init(args=args)

    node = repeater()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
