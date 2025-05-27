#!/usr/bin/env python3
# filepath: /repos/sereact_imitation/overlay_ws/src/aruco_cube_tracking/aruco_cube_tracking/aruco_cube_tracking.py
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseArray, Pose
from moveit_msgs.srv import ServoCommandType
import tf_transformations
import math
from ament_index_python.packages import get_package_share_directory
import os


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


class ArucoCubeTrackingNode(Node):
    def __init__(self):
        super().__init__('aruco_cube_tracking_node')
        
        # Declare parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('aruco_dict_type', cv2.aruco.DICT_4X4_50)
        self.declare_parameter('marker_size', 0.04)  # 40mm
        self.declare_parameter('cube_size', 0.06)  # 60mm
        self.declare_parameter('publish_rate', 30.0)  # Hz
        self.declare_parameter('calibration_matrix_path', '')
        self.declare_parameter('distortion_coefficients_path', '')
        self.declare_parameter('publish_images', False)
        self.declare_parameter('flip_horizontal', False)
        self.declare_parameter('flip_vertical', False)
        self.declare_parameter('left_cube_pose_topic', '/left/servo_node/arm_pose_cmds')
        self.declare_parameter('right_cube_pose_topic', '/right/servo_node/arm_pose_cmds')
        self.declare_parameter('left_servo_service', '/left/servo_node/switch_command_type')
        self.declare_parameter('right_servo_service', '/right/servo_node/switch_command_type')
        self.declare_parameter('teleop_mode', 'pose')
        
        # Get parameters
        self.camera_id = self.get_parameter('camera_id').value
        self.aruco_dict_type = self.get_parameter('aruco_dict_type').value
        self.marker_size = self.get_parameter('marker_size').value
        self.cube_size = self.get_parameter('cube_size').value
        self.publish_rate = self.get_parameter('publish_rate').value
        calibration_matrix_path = self.get_parameter('calibration_matrix_path').value
        distortion_coefficients_path = self.get_parameter('distortion_coefficients_path').value
        self.publish_images = self.get_parameter('publish_images').value
        self.flip_horizontal = self.get_parameter('flip_horizontal').value
        self.flip_vertical = self.get_parameter('flip_vertical').value
        left_cube_pose_topic = self.get_parameter('left_cube_pose_topic').get_parameter_value().string_value
        right_cube_pose_topic = self.get_parameter('right_cube_pose_topic').get_parameter_value().string_value
        left_servo_service = self.get_parameter('left_servo_service').get_parameter_value().string_value
        right_servo_service = self.get_parameter('right_servo_service').get_parameter_value().string_value
        self.teleop_mode = self.get_parameter('teleop_mode').get_parameter_value().string_value
        
        # Load camera calibration
        if not calibration_matrix_path:
            package_path = get_package_share_directory('aruco_cube_tracking')
            calibration_matrix_path = os.path.join(package_path, "config/calibration_matrix.npy")
            distortion_coefficients_path = os.path.join(package_path, "config/distortion_coefficients.npy")
            
        try:
            self.get_logger().info(f"Loading calibration from {calibration_matrix_path}")
            self.camera_matrix = np.load(calibration_matrix_path)
            self.distortion_coefficients = np.load(distortion_coefficients_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration: {e}")
            # Use a default calibration (will be less accurate)
            self.camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]], dtype=np.float32)
            self.distortion_coefficients = np.zeros(5, dtype=np.float32)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize camera
        self.cap = cv2.VideoCapture(self.camera_id)
        # self.cap = cv2.VideoCapture("http://10.108.248.3:8080/video")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.image_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.image_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        # Initialize ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_dict_type)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Setup marker-to-cube transforms
        self.setup_marker_to_cube_transforms()
        
        # Initialize Kalman filters for tracking
        self.left_hand_filter = KalmanFilter3D()
        self.right_hand_filter = KalmanFilter3D()
        
        # Create publishers
        self.left_cube_pub = self.create_publisher(PoseStamped, left_cube_pose_topic, 10)
        self.right_cube_pub = self.create_publisher(PoseStamped, right_cube_pose_topic, 10)
        
        # Add these publishers after the existing ones
        self.left_markers_pose_array_pub = self.create_publisher(PoseArray, 'left_hand_markers', 10)
        self.right_markers_pose_array_pub = self.create_publisher(PoseArray, 'right_hand_markers', 10)
        
        
        self.left_servo_client = self.create_client(ServoCommandType, left_servo_service)
        self.right_servo_client = self.create_client(ServoCommandType, right_servo_service)
        while not self.left_servo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /left/servo_node/switch_command_type service...')
        while not self.right_servo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /right/servo_node/switch_command_type service...')
            
        # Call the services
        self.switch_command_type()
        
        if self.publish_images:
            self.image_pub = self.create_publisher(Image, 'aruco_detections', 10)

        # Initialize TF broadcasters
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Publish the static camera transform
        self.publish_camera_transform()

        # Create timer for processing frames
        self.timer = self.create_timer(1.0 / self.publish_rate, self.process_frame)
        
        self.get_logger().info("ArUco cube tracking node initialized")
    
    
    def switch_command_type(self):
        request = ServoCommandType.Request()
        request.command_type = 2 if self.teleop_mode == 'pose' else 1

        self.left_servo_client.call_async(request)
        self.right_servo_client.call_async(request)
    
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
    
    def setup_marker_to_cube_transforms(self):
        """
        Setup transforms from each marker to the cube origin.
        For a cube of side 1, the origin is at (0.5, 0.5, 0.5)
        """
        half_side = self.cube_size / 2.0
        
        # Define transforms from each marker to cube origin
        # Each tuple is (translation, rotation_matrix)
        self.marker_to_cube = {
            0: (np.array([0, 0, -half_side]), 
                tf_transformations.quaternion_from_euler(0, 0, 0, axes='rxyz')),
            
            1: (np.array([0, 0, -half_side]), 
                tf_transformations.quaternion_from_euler(-math.pi/2, 0, 0, axes='rxyz')),
            
            2: (np.array([0, 0, -half_side]), 
                tf_transformations.quaternion_from_euler(math.pi, 0, 0, axes='rxyz')),
            
            3: (np.array([0, 0, -half_side]), 
                tf_transformations.quaternion_from_euler(math.pi/2, 0, 0, axes='rxyz')),
            
            4: (np.array([0, 0, -half_side]), 
                tf_transformations.quaternion_from_euler(math.pi/2, math.pi, 0, axes='ryxz')),
            
            5: (np.array([0, 0, -half_side]), 
                tf_transformations.quaternion_from_euler(-math.pi/2, math.pi, 0, axes='ryxz')),
        }
        
        diagonal = tf_transformations.quaternion_from_euler(0, 3*math.pi/4, math.pi/4, axes='rzyx')
        
        for i in range(6):
            cube_translation, cube_quaternion = self.marker_to_cube[i]
            cube_quaternion = tf_transformations.quaternion_multiply(cube_quaternion, diagonal)
            cube_rotation_matrix = tf_transformations.quaternion_matrix(cube_quaternion)
            cube_transform = np.eye(4)
            cube_transform[:3, :3] = cube_rotation_matrix[:3, :3]
            cube_transform[:3, 3] = cube_translation
            self.marker_to_cube[i] = cube_transform
            
        # Right hand cube uses the same transforms but with different IDs (6-11)
        for i in range(6):
            self.marker_to_cube[i + 6] = self.marker_to_cube[i]
    
    def process_frame(self):
        """Process a frame from the camera"""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame")
            return
        
        # Apply flipping if required
        if self.flip_horizontal and self.flip_vertical:
            frame = cv2.flip(frame, -1)  # Both horizontally and vertically
        elif self.flip_horizontal:
            frame = cv2.flip(frame, 1)  # Horizontal
        elif self.flip_vertical:
            frame = cv2.flip(frame, 0)  # Vertical
        
        # Convert to grayscale for ArUco detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect ArUco markers
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        output_frame = frame.copy()
        
        if ids is not None and len(ids) > 0:
            # Group markers by left/right hand
            left_hand_markers = []
            right_hand_markers = []
            
            # Draw markers and estimate poses
            if self.publish_images:
                cv2.aruco.drawDetectedMarkers(output_frame, corners, ids)
            
            left_hand_pos = None
            left_hand_quat = None
            right_hand_pos = None
            right_hand_quat = None
            
            for i, marker_id in enumerate(ids.flatten()):
                # Estimate pose
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i], self.marker_size, self.camera_matrix, self.distortion_coefficients
                )
                
                if self.publish_images:
                    # Draw axis for each marker
                    cv2.drawFrameAxes(output_frame, self.camera_matrix, self.distortion_coefficients, 
                                    rvec, tvec, self.marker_size)
                
                # Convert rotation vector to rotation matrix
                rot_matrix, _ = cv2.Rodrigues(rvec[0][0])
                
                # Get marker to cube transform
                if marker_id in self.marker_to_cube:                    
                    # Convert marker pose to cube pose
                    marker_transform = np.eye(4)
                    marker_transform[:3, :3] = rot_matrix
                    marker_transform[:3, 3] = tvec[0][0]
                    # cube_pos = marker_transform[:3, 3]
                    # cube_quat = tf_transformations.quaternion_from_matrix(marker_transform)
                    
                    cube_transform = self.marker_to_cube[marker_id]
                    cube_pose = marker_transform @ cube_transform
                    
                    cube_pos = cube_pose[:3, 3]
                    cube_quat = tf_transformations.quaternion_from_matrix(cube_pose)
                    
                    # Group by marker ID
                    if 0 <= marker_id <= 5:  # Left hand
                        left_hand_markers.append((marker_id, cube_pos, cube_quat))
                    elif 6 <= marker_id <= 11:  # Right hand
                        right_hand_markers.append((marker_id, cube_pos, cube_quat))
            
            # # Publish all detected marker poses as PoseArrays
            # if left_hand_markers:
            #     self.publish_marker_pose_array(self.left_markers_pose_array_pub, left_hand_markers)
            # if right_hand_markers:
            #     self.publish_marker_pose_array(self.right_markers_pose_array_pub, right_hand_markers)
    
            # Update left hand cube position if any markers were detected
            if left_hand_markers:
                # Average positions from all markers
                avg_position = np.mean([marker[1] for marker in left_hand_markers], axis=0)
                
                # Average orientations (quaternions)
                avg_quaternion = np.mean([marker[2] for marker in left_hand_markers], axis=0)
                avg_quaternion = avg_quaternion / np.linalg.norm(avg_quaternion)  # Normalize
                
                # Update filter with averaged pose
                left_hand_pos, left_hand_quat = self.left_hand_filter.update(
                    avg_position, avg_quaternion
                )
                # Publish pose
                self.publish_cube_pose(self.left_cube_pub, left_hand_pos, left_hand_quat, frame_id="camera_frame")
            
            # Update right hand cube position if any markers were detected
            if right_hand_markers:
                # Average positions from all markers
                avg_position = np.mean([marker[1] for marker in right_hand_markers], axis=0)
                
                # Average orientations (quaternions)
                avg_quaternion = np.mean([marker[2] for marker in right_hand_markers], axis=0)
                avg_quaternion = avg_quaternion / np.linalg.norm(avg_quaternion)  # Normalize
                
                # Update filter with averaged pose
                right_hand_pos, right_hand_quat = self.right_hand_filter.update(
                    avg_position, avg_quaternion
                )
                # Publish pose
                self.publish_cube_pose(self.right_cube_pub, right_hand_pos, right_hand_quat, frame_id="camera_frame")
        
        # Publish image if enabled
        if self.publish_images:
            img_msg = self.bridge.cv2_to_imgmsg(output_frame, encoding="bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera_frame"
            self.image_pub.publish(img_msg)
    
    # def publish_marker_pose_array(self, publisher, marker_list, frame_id="camera_frame"):
    #     """Publish a pose array containing all detected marker poses"""
    #     pose_array = PoseArray()
    #     pose_array.header.stamp = self.get_clock().now().to_msg()
    #     pose_array.header.frame_id = frame_id
        
    #     for (marker_id, position, quaternion) in marker_list:
    #         pose = Pose()
            
    #         pose.position.x = float(position[0])
    #         pose.position.y = float(position[1])
    #         pose.position.z = float(position[2])
    #         pose.orientation.x = float(quaternion[0])
    #         pose.orientation.y = float(quaternion[1])
    #         pose.orientation.z = float(quaternion[2])
    #         pose.orientation.w = float(quaternion[3])
            
    #         pose_array.poses.append(pose)
        
    #     publisher.publish(pose_array)
    
    def publish_cube_pose(self, publisher, position, orientation, frame_id="camera_frame"):
        """Publish cube pose message"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = frame_id
        
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation.x = float(orientation[0])
        pose.pose.orientation.y = float(orientation[1])
        pose.pose.orientation.z = float(orientation[2])
        pose.pose.orientation.w = float(orientation[3])
        
        publisher.publish(pose)
    
    def destroy_node(self):
        """Clean up resources on node destruction"""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoCubeTrackingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()