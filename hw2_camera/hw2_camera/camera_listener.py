import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped



class CameraListenNode(Node):

    def __init__(self):
        super().__init__('camera_listen_node')

        # Subscribe april pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/april_poses',
            self.pose_callback,
            10
        )
    
    def pose_callback(self,msg):
        # Parse marker ID from the frame_id in the header
        marker_id = msg.header.frame_id

        # Extract position data
        pos_x = msg.pose.position.x
        pos_y = msg.pose.position.y
        pos_z = msg.pose.position.z

        # Extract orientation data
        ori_x = msg.pose.orientation.x
        ori_y = msg.pose.orientation.y
        ori_z = msg.pose.orientation.z
        ori_w = msg.pose.orientation.w

        # Log the parsed data
        self.get_logger().info(f"Received Pose for Marker ID: {marker_id}")
        self.get_logger().info(f"Position - x: {pos_x}, y: {pos_y}, z: {pos_z}")
        self.get_logger().info(f"Orientation - x: {ori_x}, y: {ori_y}, z: {ori_z}, w: {ori_w}")



def main(args=None):
    rclpy.init(args=args)
    node = CameraListenNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
