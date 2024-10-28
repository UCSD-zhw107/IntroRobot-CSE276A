import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped



class CameraListenNode(Node):

    def __init__(self):
        super().__init__('camera_listen_node')

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )

        # 订阅标签姿态话题
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/april_poses',
            self.pose_callback,
            10
        )


def main():
    print('Hi from hw2_camera.')


if __name__ == '__main__':
    main()
