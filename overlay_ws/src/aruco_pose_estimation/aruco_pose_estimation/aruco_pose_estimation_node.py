import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
import cv2
import time
import os
import math
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from moveit_msgs.srv import ServoCommandType
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations
from ament_index_python.packages import get_package_share_directory


class KalmanFilter3D:
    """Simple Kalman filter for tracking 3D position and orientation"""
    
    def __init__(self, process_noise=0.01, measurement_noise=0.1):
        # State: [x, y, z, vx, vy, vz, qw, qx, qy, qz]
        self.kalman = cv2.KalmanFilter(10, 7)  # 10 state variables, 7 measurements (x,y,z,qw,qx,qy,qz)
        
        # Transition matrix (state update matrix)
        self.kalman.transitionMatrix = np.eye(10, dtype=np.float32)
        # Position + velocity model
        self.kalman.transitionMatrix[0, 3] = 1.0  # x += vx
        self.kalman.transitionMatrix[1, 4] = 1.0  # y += vy
        self.kalman.transitionMatrix[2, 5] = 1.0  # z += vz
        
        # Measurement matrix (maps state to measurement)
        self.kalman.measurementMatrix = np.zeros((7, 10), dtype=np.float32)
        self.kalman.measurementMatrix[0, 0] = 1.0  # x
        self.kalman.measurementMatrix[1, 1] = 1.0  # y
        self.kalman.measurementMatrix[2, 2] = 1.0  # z
        self.kalman.measurementMatrix[3, 6] = 1.0  # qw
        self.kalman.measurementMatrix[4, 7] = 1.0  # qx
        self.kalman.measurementMatrix[5, 8] = 1.0  # qy
        self.kalman.measurementMatrix[6, 9] = 1.0  # qz
        
        # Process noise
        self.kalman.processNoiseCov = np.eye(10, dtype=np.float32) * process_noise
        # Higher process noise for velocities
        self.kalman.processNoiseCov[3:6, 3:6] *= 10
        
        # Measurement noise
        self.kalman.measurementNoiseCov = np.eye(7, dtype=np.float32) * measurement_noise
        
        # Initial state covariance
        self.kalman.errorCovPost = np.eye(10, dtype=np.float32) * 1.0
        
        self.initialized = False
        
    def update(self, position, orientation):
        """Update the filter with a new measurement"""
        measurement = np.array([
            position[0], position[1], position[2],
            orientation[0], orientation[1], orientation[2], orientation[3]
        ], dtype=np.float32).reshape(-1, 1)
        
        if not self.initialized:
            # Initialize state
            self.kalman.statePost[0:3, 0] = position
            self.kalman.statePost[3:6, 0] = 0  # Initial velocity is zero
            self.kalman.statePost[6:10, 0] = orientation
            self.initialized = True
            return position, orientation
            
        # Predict
        predicted = self.kalman.predict()
        
        # Update with measurement
        corrected = self.kalman.correct(measurement)
        
        # Extract position and orientation
        filtered_pos = corrected[0:3, 0]
        filtered_quat = corrected[6:10, 0]
        
        # Normalize quaternion
        quat_norm = np.linalg.norm(filtered_quat)
        if quat_norm > 0:
            filtered_quat = filtered_quat / quat_norm
            
        return filtered_pos, filtered_quat


class ArucoPoseEstimationNode(Node):
    def __init__(self):
        super().__init__('aruco_pose_estimation_node')
        
        # Declare parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('aruco_dict_type', 0)  # Default: DICT_4X4_50
        self.declare_parameter('marker_length', 0.053)
        self.declare_parameter('publish_image', True)
        self.declare_parameter('calibration_matrix_path', "")
        self.declare_parameter('distortion_coefficients_path', "")
        self.declare_parameter('publish_rate', 30.0)  # Hz
        self.declare_parameter('flip_horizontal', False)
        self.declare_parameter('flip_vertical', False)
        self.declare_parameter('frame_id', 'world')  # Default frame_id for the published poses
        self.declare_parameter('left_marker_id', 0)  # Default marker ID for left marker
        self.declare_parameter('right_marker_id', 1)  # Default marker ID for right marker
        self.declare_parameter('left_cube_pose_topic', '/left/servo_node/arm_pose_cmds')
        self.declare_parameter('right_cube_pose_topic', '/right/servo_node/arm_pose_cmds')
        self.declare_parameter('left_gripper_action', '/left_hand_controller/gripper_cmd')
        self.declare_parameter('right_gripper_action', '/right_hand_controller/gripper_cmd')
        self.declare_parameter('left_servo_service', '/left/servo_node/switch_command_type')
        self.declare_parameter('right_servo_service', '/right/servo_node/switch_command_type')
        self.declare_parameter('teleop_mode', 'pose')
        self.declare_parameter('gripper_close_position', 0.0)
        self.declare_parameter('gripper_open_position', 0.8)
        self.declare_parameter('gripper_max_effort', 100.0)
        self.declare_parameter('gripper_trigger_angle', 0.5)  # Threshold for gripper action
        
        # Get parameters
        self.camera_id = self.get_parameter('camera_id').value
        self.aruco_dict_type = self.get_parameter('aruco_dict_type').value
        self.marker_length = self.get_parameter('marker_length').value
        self.publish_image = self.get_parameter('publish_image').value
        calibration_matrix_path = self.get_parameter('calibration_matrix_path').value
        distortion_coefficients_path = self.get_parameter('distortion_coefficients_path').value
        publish_rate = self.get_parameter('publish_rate').value
        self.flip_horizontal = self.get_parameter('flip_horizontal').value
        self.flip_vertical = self.get_parameter('flip_vertical').value
        self.frame_id = self.get_parameter('frame_id').value
        self.left_marker_id = self.get_parameter('left_marker_id').value
        self.right_marker_id = self.get_parameter('right_marker_id').value
        left_cube_pose_topic = self.get_parameter('left_cube_pose_topic').get_parameter_value().string_value
        right_cube_pose_topic = self.get_parameter('right_cube_pose_topic').get_parameter_value().string_value
        left_gripper_action = self.get_parameter('left_gripper_action').get_parameter_value().string_value
        right_gripper_action = self.get_parameter('right_gripper_action').get_parameter_value().string_value
        left_servo_service = self.get_parameter('left_servo_service').get_parameter_value().string_value
        right_servo_service = self.get_parameter('right_servo_service').get_parameter_value().string_value
        self.teleop_mode = self.get_parameter('teleop_mode').get_parameter_value().string_value
        self.gripper_close_position = self.get_parameter('gripper_close_position').value
        self.gripper_open_position = self.get_parameter('gripper_open_position').value
        self.gripper_max_effort = self.get_parameter('gripper_max_effort').value
        self.gripper_trigger_angle = self.get_parameter('gripper_trigger_angle').value
        self.prev_marker_angle_left = None
        self.prev_marker_angle_right = None
        
        
        self.get_logger().info(f"left marker ID: {self.left_marker_id}")
        self.get_logger().info(f"right marker ID: {self.right_marker_id}")

        # Load calibration data
        package_path = get_package_share_directory('aruco_pose_estimation')
        
        # If paths are empty, try default locations
        if not calibration_matrix_path:
            calibration_matrix_path = os.path.join(package_path, "config/calibration_matrix.npy")
        if not distortion_coefficients_path:
            distortion_coefficients_path = os.path.join(package_path, "config/distortion_coefficients.npy")
            
        try:
            self.get_logger().info(f"Loading calibration from {calibration_matrix_path}")
            self.k = np.load(calibration_matrix_path)
            self.d = np.load(distortion_coefficients_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration data: {e}")
            raise

        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create publishers
        self.left_pose_publisher = self.create_publisher(PoseStamped, left_cube_pose_topic, 10)
        self.right_pose_publisher = self.create_publisher(PoseStamped, right_cube_pose_topic, 10)
        
        self.left_hand = ActionClient(self, GripperCommand, left_gripper_action)
        self.right_hand = ActionClient(self, GripperCommand, right_gripper_action)
        
        self.left_servo_client = self.create_client(ServoCommandType, left_servo_service)
        self.right_servo_client = self.create_client(ServoCommandType, right_servo_service)
        
        while not self.left_servo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /left/servo_node/switch_command_type service...')
        while not self.right_servo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /right/servo_node/switch_command_type service...')
            
        # Call the services
        self.switch_command_type()
        
        if self.publish_image:
            self.image_publisher = self.create_publisher(Image, 'aruco_image', 10)
        
        # Initialize transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize TF broadcasters
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish the static camera transform
        self.publish_camera_transform()
        
        # Initialize Kalman filters for tracking
        self.left_hand_filter = KalmanFilter3D()
        self.right_hand_filter = KalmanFilter3D()
        
        # fixed transform
        self.marker_to_gripper = tf_transformations.quaternion_from_euler(-math.pi/2, -math.pi, 0, axes='rxyz')
        
        # Initialize camera
        self.video = cv2.VideoCapture(self.camera_id)
        self.video.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.video.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        time.sleep(2.0)  # Camera warm-up
        
        if not self.video.isOpened():
            self.get_logger().error("Failed to open camera!")
            return
            
        # Create timer
        self.timer = self.create_timer(1.0/publish_rate, self.timer_callback)
        
        self.get_logger().info("ArUco pose estimation node initialized")


    def switch_command_type(self):
        request = ServoCommandType.Request()
        request.command_type = 2 if self.teleop_mode == 'pose' else 1

        left_future = self.left_servo_client.call_async(request)
        rclpy.spin_until_future_complete(self, left_future)
        right_future = self.right_servo_client.call_async(request)
        rclpy.spin_until_future_complete(self, right_future)
        
    def send_goal(self, position=0.8, effort=100.0, hand=""):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = effort

        self.get_logger().info(f'Sending goal: position={position}, effort={effort}')
        if hand == "left":
            self.left_hand.send_goal_async(goal_msg)
        else:
            self.right_hand.send_goal_async(goal_msg)

    def publish_camera_transform(self):
        """Publish static transform from world frame to camera frame"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "world"
        transform.child_frame_id = "camera_frame"
        
        # Position the camera at a reasonable default location
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.3  # 1 meter above world origin
        
        # Orientation: camera facing forward along the x-axis
        # This rotation corresponds to camera z-axis pointing forward
        quaternion = tf_transformations.quaternion_from_euler(-math.pi/2, 0, -math.pi/2)  # Rotated 90° around Y
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        
        # Publish the transform
        self.static_broadcaster.sendTransform(transform)
    
    def timer_callback(self):
        ret, frame = self.video.read()
        if not ret:
            self.get_logger().error("Failed to capture frame")
            return
            
        # Detect markers and estimate poses
        output_frame, rvecs, tvecs, ids = self.pose_estimation(
            frame, 
            self.aruco_dict_type, 
            self.marker_length,
            self.k, 
            self.d
        )
        
        # Create and publish pose array
        if len(ids) > 0:
            # For each marker, broadcast transform and add to pose array
            for i in range(len(ids)):
                marker_id = ids[i]
                rvec = rvecs[i]
                tvec = tvecs[i]
                
                # Convert rotation vector to quaternion
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                rotation_matrix_3x3 = np.eye(4)
                rotation_matrix_3x3[:3, :3] = rotation_matrix
                rotation_matrix_3x3[:3, 3] = tvec
                marker_q = tf_transformations.quaternion_from_matrix(rotation_matrix_3x3)
                gripper_orientation = tf_transformations.euler_from_quaternion(marker_q)
                marker_q = tf_transformations.quaternion_multiply(marker_q, self.marker_to_gripper)
                
                if marker_id == self.left_marker_id:
                    tvec, _ = self.left_hand_filter.update(tvec, [0.0, 0.75, 0.75, 0.0])
                if marker_id == self.right_marker_id:
                    tvec, _ = self.right_hand_filter.update(tvec, [0.0, 0.75, 0.75, 0.0])
                
                # Create pose
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "camera_frame"
                pose.pose.position.x = float(tvec[0])
                pose.pose.position.y = float(tvec[1])
                pose.pose.position.z = float(tvec[2])
                pose.pose.orientation.x = 0.0 # marker_q[0]
                pose.pose.orientation.y = 0.75 # marker_q[1]
                pose.pose.orientation.z = 0.75 # marker_q[2]
                pose.pose.orientation.w = 0.0 # marker_q[3]
                
                if marker_id == self.left_marker_id:
                    self.left_pose_publisher.publish(pose)
                    if self.prev_marker_angle_left is not None:
                        if self.prev_marker_angle_left <= self.gripper_trigger_angle and gripper_orientation[2] > self.gripper_trigger_angle:
                            self.send_goal(self.gripper_open_position, self.gripper_max_effort, hand="left")
                        elif self.prev_marker_angle_left > self.gripper_trigger_angle and gripper_orientation[2] <= self.gripper_trigger_angle:
                            self.send_goal(self.gripper_close_position, self.gripper_max_effort, hand="left")
                    self.prev_marker_angle_left = gripper_orientation[2]
                    
                elif marker_id == self.right_marker_id:
                    self.right_pose_publisher.publish(pose)
                    if self.prev_marker_angle_right is not None:
                        if self.prev_marker_angle_right <= self.gripper_trigger_angle and gripper_orientation[2] > self.gripper_trigger_angle:
                            self.send_goal(self.gripper_open_position, self.gripper_max_effort, hand="right")
                        elif self.prev_marker_angle_right > self.gripper_trigger_angle and gripper_orientation[2] <= self.gripper_trigger_angle:
                            self.send_goal(self.gripper_close_position, self.gripper_max_effort, hand="right")
                    self.prev_marker_angle_right = gripper_orientation[2]
        
        # Publish image if enabled
        if self.publish_image and output_frame is not None:
            
            # Apply flipping based on parameters
            if self.flip_horizontal and self.flip_vertical:
                output_frame = cv2.flip(output_frame, -1)  # Flip both horizontally and vertically
            elif self.flip_horizontal:
                output_frame = cv2.flip(output_frame, 1)   # Flip horizontally
            elif self.flip_vertical:
                output_frame = cv2.flip(output_frame, 0)   # Flip vertically
                
            img_msg = self.bridge.cv2_to_imgmsg(output_frame, encoding="bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera_frame"
            self.image_publisher.publish(img_msg)

    def pose_estimation(self, frame, aruco_dict_type, marker_length, matrix_coefficients, distortion_coefficients):
        rvec = []
        tvec = []
        idx = []
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        parameters = cv2.aruco.DetectorParameters()

        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict, parameters=parameters)

        # If markers are detected
        if len(corners) > 0 and ids is not None:
            for i in range(0, len(ids)):
                # Estimate pose of each marker and return the values rvec and tvec
                r, t, markerPoints = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i], 
                    marker_length, 
                    matrix_coefficients,
                    distortion_coefficients
                )
                rvec.append(r[0][0])
                tvec.append(t[0][0])
                idx.append(ids[i][0])
                
                if self.publish_image:
                    # Draw a square around the markers
                    cv2.aruco.drawDetectedMarkers(frame, corners) 

                    # Draw Axis
                    cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, r, t, marker_length*2)  
        
        rvec = np.array(rvec)
        tvec = np.array(tvec)
        return frame, rvec, tvec, idx

    def destroy_node(self):
        if hasattr(self, 'video') and self.video.isOpened():
            self.video.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseEstimationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()