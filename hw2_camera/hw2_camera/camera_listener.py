import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from hw2_camera.key_parser import get_key, restore_terminal_settings, save_terminal_settings
from hw2_camera.camera_control import Camera_control


class CameraListenNode(Node):

    def __init__(self):
        super().__init__('camera_listen_node')

        #Controller
        self.controller = Camera_control()

        # Subscribe to April Tag
        self.pose_subscriber = None  
        self.closest_marker = None
        self.closest_distance = float('inf')  
        self.is_listening = False 
        self.last_label_pose = None
        self.detection_time = 10.0
        self.label_loc = {}

        #Publish to Kinematic Model
        self.publisher_ = self.create_publisher(Joy, '/joy', 1)
        self.waypoints = None
        self.current_pose = None
        self.settings = save_terminal_settings()

    def set_label_location(self,label):
        self.label_loc = label

    def split_waypoint(self,waypoints,step_size=0.2):
        continuous_points = []
        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            target = waypoints[i + 1]
            x_start, y_start, theta_start = start
            x_target, y_target, theta_target = target

            # Adjust initial orientation to face target
            delta_x = x_target - x_start
            delta_y = y_target - y_start
            target_orientation = math.atan2(delta_y, delta_x)

            # Add initial orientation adjustment point
            continuous_points.append((x_start, y_start, target_orientation))

            # Move in steps of 20 cm along the orientation
            distance = math.sqrt(delta_x**2 + delta_y**2)
            num_steps = int(distance // step_size)

            for step in range(1, num_steps + 1):
                new_x = x_start + step * step_size * math.cos(target_orientation)
                new_y = y_start + step * step_size * math.sin(target_orientation)
                continuous_points.append((new_x, new_y, target_orientation))
            continuous_points.append((x_target, y_target, theta_target))
        self.waypoints = continuous_points
        self.current_pose = continuous_points[0]

    # Start Listen
    def start_listening(self):
        self.closest_marker = None
        self.closest_distance = float('inf')
        self.is_listening = True
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/april_poses',
            self.pose_callback,
            10
        )
        self.get_logger().info("Listening for AprilTag for 10 seconds...")
        
        # Listen 10s and stop
        self.create_timer(self.detection_time, self.stop_listening)

    # Stop Listen
    def stop_listening(self):
        if self.is_listening:
            self.destroy_subscription(self.pose_subscriber)
            self.is_listening = False

            if self.closest_marker:
                marker_id = self.closest_marker.header.frame_id
                pos_x = self.closest_marker.pose.position.x
                pos_y = self.closest_marker.pose.position.y
                pos_z = self.closest_marker.pose.position.z
                ori_x = self.closest_marker.pose.orientation.x
                ori_y = self.closest_marker.pose.orientation.y
                ori_z = self.closest_marker.pose.orientation.z
                ori_w = self.closest_marker.pose.orientation.w

                #self.get_logger().info(f"Closest Marker ID: {marker_id} at distance: {self.closest_distance:.2f} cm")
                #self.get_logger().info(f"Position - x: {pos_x}, y: {pos_y}, z: {pos_z}")
                #self.get_logger().info(f"Orientation - x: {ori_x}, y: {ori_y}, z: {ori_z}, w: {ori_w}")
                self.get_logger().info('April Tag Detected')
                pos = [pos_x,pos_y,pos_z]
                ori = [ori_x,ori_y,ori_z,ori_w]
                self.last_label_pose = {'id': int(marker_id), 'pose': pos, 'orientation': ori}
            else:
                self.get_logger().info("No AprilTag detected during listening period.")
                self.last_label_pose = None

            self.get_logger().info("Resuming movement...")
            
    
    # April Tag Detection
    def pose_callback(self,msg):
        distance = math.sqrt(msg.pose.position.x**2 + msg.pose.position.z**2)
        
        # Closest marker
        if distance < self.closest_distance:
            self.closest_distance = distance
            self.closest_marker = msg

    # Get Last Label
    def get_label(self):
        return self.last_label_pose
    

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
    
    # Control Loop
    def controlLoop(self):
        for target in self.waypoints:
            #pos_error, ori_error = self.controller.pose_error(self.current_pose,target)
            #if(pos_error < 0.15 and abs(ori_error) < math.radians(20)):
                #print("Skipped")
                #continue
            # Get vision feedback
            self.start_listening()
            while self.is_listening:
                rclpy.spin_once(self, timeout_sec=0.5)

            # Compute robot pose
            if self.last_label_pose != None:
                label_id = self.last_label_pose['id']
                label_pose = self.last_label_pose['pose']
                label_ori = self.last_label_pose['orientation']
                label_pose_world = self.label_loc[label_id]
                self.current_pose = self.controller.computeObservedPose(label_pose,label_ori,label_pose_world)
            # Move to target
            motion = self.controller.control(self.current_pose, target)
            self.sendMotion(motion)
            time.sleep(motion[3] + 0.5)
            # Update pose
            self.current_pose = self.controller.updatePose(motion,self.current_pose)
            self.get_logger().info(f'Pose X: {self.current_pose[0]}')
            self.get_logger().info(f'Pose Y: {self.current_pose[1]}')
            self.get_logger().info(f'Pose Ori: {self.current_pose[2]}')



        return 0

    def run(self):
        while True:
            # parse keyboard control
            key = get_key(self.settings, timeout=0.1)

            # interpret keyboard control as joy
            joy_msg, flag = self.key_to_joy(key)
            if flag is False:
                break
        self.stop()

    def key_to_joy(self, key):
        flag = True
        joy_msg = Joy()
        joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        if key == 'w':
            self.controlLoop()
            flag = False
        elif key == 's':
            joy_msg.axes[4] = -1.0
            self.publisher_.publish(joy_msg)
        elif key == 'q':
            flag = False
        elif (len(key) > 0 and ord(key) == 27) or (key == '\x03'):
            flag = False
        return joy_msg, flag
    

    def stop(self):
        restore_terminal_settings(self.settings)



def main(args=None):
    rclpy.init(args=args)
    node = CameraListenNode()
    waypoints = [
        [0,0,0],
        [1,0,0],
        [1,2,math.pi/2]
    ]
    node.split_waypoint(waypoints)
    label = {
        0: [1.275, 0, math.pi],
        1: [1.0, 2.13, -(math.pi/2)],
        2: [0.55, 1.0, -(math.pi/2)]
    }
    node.set_label_location(label=label)
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()