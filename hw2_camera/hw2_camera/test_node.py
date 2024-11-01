import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from hw2_camera.camera_control import Camera_control

import time
import math

class WaypointControlNode(Node):
    def __init__(self):
        super().__init__('waypoint_control_node')

        # WayPoints
        self.waypoints = None
        self.current_waypoint_index = 0
        self.current_pose = None
        
        # Camera Reading Time
        self.wait_time_for_camera = 10.0  # 固定等待摄像头数据的时间
        self.data_collection_duration = 3.0  # 数据收集持续时间 (秒)
        
        
        
        # 订阅摄像头数据
        self.subscription = self.create_subscription(
            PoseStamped,
            '/april_poses',
            self.camera_callback,
            100
        )
        self.publisher_ = self.create_publisher(Joy, '/joy', 1)
        
        # 初始化摄像头数据缓存
        self.camera_data_list = []  # 用于缓存3秒内的所有消息
        self.received_data = False  # 标志用于检测是否收到数据
        self.label_loc = {}
        self.controller = Camera_control()

    def camera_callback(self, msg):
        # 在3秒收集时间内，缓存摄像头数据
        self.camera_data_list.append(msg)
        self.received_data = True  # 标志设置为True，表示收到数据

    def set_label_location(self,label):
        self.label_loc = label

    def set_way_point(self,waypoints):
        self.current_pose = waypoints[0]
        self.waypoints = waypoints[1:]

    # Publish to Kinematic model
    def sendMotion(self, motion):
        joy_msg = Joy()
        joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        vx = motion[0]
        vy = motion[1]
        wz = motion[2]
        t = motion[3]
        joy_msg.axes[0] = vx
        joy_msg.axes[1] = vy
        joy_msg.axes[2] = wz
        joy_msg.axes[3] = t
        joy_msg.axes[4] = 1
        self.publisher_.publish(joy_msg)

    def run(self):
        while self.current_waypoint_index < len(self.waypoints):
            
            # 当前航点
            current_waypoint = self.waypoints[self.current_waypoint_index]
            self.get_logger().warn(f'Target: {current_waypoint}')

            # 等待10秒让摄像头数据更新
            self.get_logger().info('Waiting for 10 seconds before receiving camera data...')
            time.sleep(self.wait_time_for_camera)

            # 开始3秒数据收集
            self.get_logger().info('Collecting camera data for 3 seconds...')
            self.camera_data_list.clear()  # 清空之前缓存的数据
            self.received_data = False  # 重置标志
            start_time = time.time()
            
            while time.time() - start_time < self.data_collection_duration:
                # 持续处理回调，以确保数据接收
                rclpy.spin_once(self, timeout_sec=0.1)

            # 检查是否收到了任何数据
            if self.received_data:
                closest_marker = self.find_closest_marker(self.camera_data_list)
                self.get_logger().info(f'Closest marker found with id: {closest_marker.header.frame_id}')
                
                # 使用距离最近的标记信息进行控制
                self.compute_control(current_waypoint, closest_marker)
                
            else:
                # 如果没有接收到数据
                self.get_logger().warn('No camera data received during data collection period.')

    def find_closest_marker(self, camera_data_list):
        # 找到距离最近的标记
        min_distance = float('inf')
        closest_marker = None
        for msg in camera_data_list:
            distance = math.sqrt(msg.pose.position.x**2 + msg.pose.position.z**2)
            if distance < min_distance:
                min_distance = distance
                closest_marker = msg
        return closest_marker

    def compute_control(self, target, closest_marker):
        # 根据航点和摄像头数据计算控制输出
        pos_x = closest_marker.pose.position.x
        pos_y = closest_marker.pose.position.y
        pos_z = closest_marker.pose.position.z
        ori_x = closest_marker.pose.orientation.x
        ori_y = closest_marker.pose.orientation.y
        ori_z = closest_marker.pose.orientation.z
        ori_w = closest_marker.pose.orientation.w
        
        # 假设计算得到的控制输出值
        label_pose = [pos_x,pos_y,pos_z]
        label_ori = [ori_x,ori_y,ori_z,ori_w]
        label_id = closest_marker.header.frame_id
        label_pose_world = self.label_loc[int(label_id)]
        self.current_pose = self.controller.computeObservedPose(label_pose,label_ori,label_pose_world)
        self.get_logger().info(f'Pose X: {self.current_pose[0]}')
        self.get_logger().info(f'Pose Y: {self.current_pose[1]}')
        self.get_logger().info(f'Pose Ori: {self.current_pose[2]}')
        # Error
        state = None
        pos_error, ori_error = self.controller.pose_error(self.current_pose, target)
        if target == [1,0,math.pi/2]:
            state = 'rotation'
            if ori_error < math.radians(10):
                self.current_waypoint_index += 1
                self.get_logger().warn('Next Target')
                return
        else:
            state = 'translation'
            if pos_error < 0.16 and ori_error < math.radians(10):
                self.current_waypoint_index += 1
                self.get_logger().warn('Next Target')
                return
        dt = 0.5
        motion = self.controller.control(self.current_pose, target, dt, state)
        self.sendMotion(motion)
        time.sleep(motion[3] + 0.5)


def main(args=None):
    rclpy.init(args=args)
    waypoint_control_node = WaypointControlNode()
    waypoints = [
        [0,0,0],
        [1,0,0],
        [1,0,math.pi/2],
        [1,1,math.pi/2],
        [1,2,math.pi/2]
    ]
    waypoint_control_node.set_way_point(waypoints)
    label = {
        5: [1.275, 0, math.pi],
        1: [1.0, 2.16, -(math.pi/2)],
        2: [0.65, 1.0, -(math.pi/2)]
    }
    '''
    0: [0.0, -0.107, math.pi/2],
    1: [1.0, 2.26, -(math.pi/2)],
    2: [0.13, 1.21, 0],
    5: [1.275, 0, -math.pi]
    '''
    waypoint_control_node.set_label_location(label=label)
    waypoint_control_node.run()  # 使用run函数启动控制逻辑
    waypoint_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
