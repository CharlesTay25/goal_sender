import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

class GoalSenderNode(Node):
    LOCATIONS = [
        {'name': "Home", 'pose': (-0.040695905685424805, -1.067751407623291, 0.0, 0.0, 0.0, 0.0049739674741855165, 0.9999876297472713)},
        {'name': "Row 1", 'pose': (-5.62584, -1.28362, 0.0, 0.0, 0.0, -0.00452601, 0.99999)},
        {'name': "Row 2", 'pose': (-5.170678615570068, 0.004885086532795252, 0.0, 0.0, 0.0, -0.00452601, 0.999988067893596 )},
    ]

    def __init__(self):
        super().__init__('goal_sender_node')
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.publisher = self.create_publisher(PoseStamped, '/i069/goal_pose', qos_profile)
        self.goal_subscriber = self.create_subscription(Int32, '/preset_location', self.preset_location_callback, 10)

    def preset_location_callback(self, msg):
        if msg.data < len(self.LOCATIONS) and msg.data >= 0:
            location = self.LOCATIONS[msg.data]
            goal_msg = self.create_goal_message(location['pose'])
            self.get_logger().info(f'Going to Preset {location["name"]}: x={goal_msg.pose.position.x}, y={goal_msg.pose.position.y}, theta={goal_msg.pose.orientation.z}')
            self.publisher.publish(goal_msg)
        elif msg.data == -10:
            pass
        else:
            self.get_logger().warn(f'Invalid location index received: {msg.data}')

    def create_goal_message(self, pose):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x, goal_msg.pose.position.y, goal_msg.pose.position.z, \
            goal_msg.pose.orientation.x, goal_msg.pose.orientation.y, goal_msg.pose.orientation.z, \
            goal_msg.pose.orientation.w = pose
        return goal_msg

def main(args=None):
    rclpy.init(args=args)
    goal_sender_node = GoalSenderNode()

    rclpy.spin(goal_sender_node)

    goal_sender_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
