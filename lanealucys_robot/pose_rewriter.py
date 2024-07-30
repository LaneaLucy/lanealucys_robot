import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Header


class pose_rewriter(Node):

    def __init__(self):
        super().__init__('pose_rewriter')
        
        #self.message = PoseWithCovarianceStamped()
        
        self.declare_parameter('input_topic', '/pose')
        self.declare_parameter('output_topic', '/pose_cov')
        self.declare_parameter('frame_id', 'odom')
        
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        self.get_logger().info('Input Topic: "%s"' % self.input_topic)
        self.get_logger().info('Output Topic: "%s"' % self.output_topic)
        self.get_logger().info('frame_id: "%s"' % self.frame_id)
        
        #print('rclpy:')
        #print(rclpy.time.Time())
        #print('node:')
        #print(Node.get_clock(self).now())
        
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, self.output_topic, rclpy.qos.qos_profile_sensor_data)
        
        self.subscription = self.create_subscription(
            PoseStamped,
            self.input_topic,
            self.listener_callback,
            rclpy.qos.qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        message = PoseWithCovarianceStamped()
        message.header = msg.header
        message.pose.pose = msg.pose
        message.pose.covariance[0] = 0.5
        message.pose.covariance[7] = 0.5
        message.pose.covariance[14] = 0.5
        message.pose.covariance[21] = 0.5
        message.pose.covariance[28] = 0.5
        message.pose.covariance[35] = 0.5
        message.header.frame_id = self.frame_id
        
        self.publisher_.publish(message)


def main(args=None):
    rclpy.init(args=args)

    node = pose_rewriter()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
