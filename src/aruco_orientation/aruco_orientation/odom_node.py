#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import math
import matplotlib.pyplot as plt
import cv2
import numpy as np
import pyzed.sl as sl

# Define constants
MARKER_IDS = [0, 1]  # List of marker IDs for both robots
ALPHA = 1.0

def ema_filter(data, previous_value, alpha=0.03):
    return alpha * data + (1 - alpha) * previous_value

class RobotTrackerNode(Node):
    def __init__(self):
        super().__init__('robot_tracker_node_multi')

        # Initialize data structures for each robot
        self.robots = {}
        for marker_id in MARKER_IDS:
            self.robots[marker_id] = {
                'yaw_buffer': np.zeros(10),
                'buffer_size': 10,
                'buffer_index': 0,
                'x_aruco': None,
                'y_aruco': None,
                'yaw_aruco': None,
                'x_odom': None,
                'y_odom': None,
                'yaw_odom': None,
                'x_offset': 0.0,
                'y_offset': 0.0,
                'theta_offset': 0.0,
                'valid_transform': False,
                'x_est': None,
                'y_est': None,
                'yaw_est': None,
                'publisher_x': None,
                'publisher_y': None,
                'publisher_yaw': None,
                'odom_subscription': None,
                'plot_color': 'r' if marker_id == 0 else 'b'  # Different colors for plotting
            }

        # Initialize publishers and subscribers for each robot
        self.initialize_publishers_and_subscribers()

        # Initialize camera and plotting
        self.init_plot()
        self.zed = self.initialize_zed_camera()

        # ArUco detection parameters
        self.camera_matrix = np.array([[1000, 0, 640], [0, 1000, 360], [0, 0, 1]], dtype=float)
        self.dist_coeffs = np.zeros((5, 1))
        resolution = sl.Resolution(1280, 720)
        self.image_zed = sl.Mat(resolution.width, resolution.height, sl.MAT_TYPE.U8_C4)
        self.depth_map = sl.Mat(resolution.width, resolution.height, sl.MAT_TYPE.F32_C1)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # Timer for detection and update
        self.timer = self.create_timer(0.1, self.detect_and_update)

    def initialize_publishers_and_subscribers(self):
        for marker_id in MARKER_IDS:
            robot = self.robots[marker_id]
            # Set publisher topic names
            if marker_id == 0:
                topic_suffix = ''
                odom_topic = '/odom'
            else:
                topic_suffix = '1'
                odom_topic = '/tb3_0/odom'

            # Initialize publishers
            robot['publisher_x'] = self.create_publisher(Float32, f'aruco{topic_suffix}_x', 10)
            robot['publisher_y'] = self.create_publisher(Float32, f'aruco{topic_suffix}_y', 10)
            robot['publisher_yaw'] = self.create_publisher(Float32, f'aruco{topic_suffix}_yaw', 10)

            # Initialize odometry subscribers
            robot['odom_subscription'] = self.create_subscription(
                Odometry, odom_topic, lambda msg, mid=marker_id: self.odom_callback(msg, mid), 10)

    def init_plot(self):
        self.fig, self.ax = plt.subplots()
        self.ax.set_title('Robot Positions and Orientations')
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        plt.ion()
        plt.show()

    def initialize_zed_camera(self):
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        init_params.coordinate_units = sl.UNIT.METER
        init_params.camera_resolution = sl.RESOLUTION.HD2K

        zed = sl.Camera()
        if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            print("Unable to open ZED Camera")
            zed.close()
            exit(1)

        positional_tracking_parameters = sl.PositionalTrackingParameters()
        positional_tracking_parameters.set_as_static = True
        zed.enable_positional_tracking(positional_tracking_parameters)

        return zed

    def get_yaw_from_aruco(self, rvec):
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        return yaw

    def odom_callback(self, msg, marker_id):
        robot = self.robots[marker_id]
        # Extract position and orientation from odometry
        robot['x_odom'] = msg.pose.pose.position.x
        robot['y_odom'] = msg.pose.pose.position.y

        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
        robot['yaw_odom'] = math.atan2(siny_cosp, cosy_cosp)

    def detect_and_update(self):
        """Detect ArUco markers and update the robots' positions and orientations."""
        rt_param = sl.RuntimeParameters()
        rt_param.measure3D_reference_frame = sl.REFERENCE_FRAME.WORLD

        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT)
            image_ocv = self.image_zed.get_data()[:, :, :3]
            image_ocv_bgr = cv2.cvtColor(image_ocv, cv2.COLOR_RGB2BGR)

            corners, ids, _ = cv2.aruco.detectMarkers(image_ocv_bgr, self.aruco_dict, parameters=self.aruco_params)

            detected_markers = set()

            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id in MARKER_IDS:
                        robot = self.robots[marker_id]
                        marker_size = 0.05
                        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                            [corners[i]], marker_size, self.camera_matrix, self.dist_coeffs)

                        robot['x_aruco'], robot['y_aruco'] = tvecs[0][0][0], -tvecs[0][0][1]
                        yaw_aruco = -self.get_yaw_from_aruco(rvecs[0])

                        # Update buffers
                        robot['yaw_buffer'][robot['buffer_index']] = yaw_aruco
                        robot['buffer_index'] = (robot['buffer_index'] + 1) % robot['buffer_size']

                        robot['yaw_aruco'] = np.mean(robot['yaw_buffer'])

                        # Publish ArUco data
                        robot['publisher_x'].publish(Float32(data=robot['x_aruco']))
                        robot['publisher_y'].publish(Float32(data=robot['y_aruco']))
                        robot['publisher_yaw'].publish(Float32(data=robot['yaw_aruco']))

                        # If odometry data is available, compute the transform
                        if (robot['x_odom'] is not None and
                            robot['y_odom'] is not None and
                            robot['yaw_odom'] is not None):
                            self.compute_transform(robot)
                            robot['valid_transform'] = True

                        detected_markers.add(marker_id)

            # For robots not detected, estimate position
            for marker_id in MARKER_IDS:
                if marker_id not in detected_markers:
                    robot = self.robots[marker_id]
                    if robot['valid_transform'] and robot['x_odom'] is not None:
                        # Use odometry data and the stored transform to estimate global position
                        self.estimate_global_position(robot)

                        # Publish estimated position
                        robot['publisher_x'].publish(Float32(data=robot['x_est']))
                        robot['publisher_y'].publish(Float32(data=robot['y_est']))
                        robot['publisher_yaw'].publish(Float32(data=robot['yaw_est']))

            self.update_plot()

    def compute_transform(self, robot):
        """Compute the transformation between the odom frame and the global frame for a robot."""
        # Compute theta_offset
        robot['theta_offset'] = robot['yaw_aruco'] - robot['yaw_odom']

        # Compute x_offset and y_offset
        cos_theta = math.cos(robot['theta_offset'])
        sin_theta = math.sin(robot['theta_offset'])
        robot['x_offset'] = robot['x_aruco'] - (robot['x_odom'] * cos_theta - robot['y_odom'] * sin_theta)
        robot['y_offset'] = robot['y_aruco'] - (robot['x_odom'] * sin_theta + robot['y_odom'] * cos_theta)

    def estimate_global_position(self, robot):
        """Estimate global position using odometry data and the stored transform for a robot."""
        cos_theta = math.cos(robot['theta_offset'])
        sin_theta = math.sin(robot['theta_offset'])
        robot['x_est'] = robot['x_offset'] + robot['x_odom'] * cos_theta - robot['y_odom'] * sin_theta
        robot['y_est'] = robot['y_offset'] + robot['x_odom'] * sin_theta + robot['y_odom'] * cos_theta
        robot['yaw_est'] = robot['theta_offset'] + robot['yaw_odom']

    def update_plot(self):
        self.ax.clear()
        self.ax.set_title('Robot Positions and Orientations')
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.grid(True)

        all_x = []
        all_y = []

        for marker_id in MARKER_IDS:
            robot = self.robots[marker_id]
            color = robot['plot_color']

            # Plot estimated position
            if robot['valid_transform'] and robot['x_est'] is not None and robot['y_est'] is not None:
                u_est = math.cos(robot['yaw_est'])
                v_est = math.sin(robot['yaw_est'])
                self.ax.quiver(robot['x_est'], robot['y_est'], u_est, v_est, angles='xy',
                               scale_units='xy', scale=1, color=color, label=f"Robot {marker_id} Estimated Position")
                self.ax.scatter(robot['x_est'], robot['y_est'], color=color)
                all_x.append(robot['x_est'])
                all_y.append(robot['y_est'])
            # Plot ArUco position
            if robot['x_aruco'] is not None and robot['y_aruco'] is not None:
                u = math.cos(robot['yaw_aruco'])
                v = math.sin(robot['yaw_aruco'])
                self.ax.quiver(robot['x_aruco'], robot['y_aruco'], u, v, angles='xy',
                               scale_units='xy', scale=1, color=color, linestyle='--', label=f"Robot {marker_id} ArUco Position")
                self.ax.scatter(robot['x_aruco'], robot['y_aruco'], color=color, marker='x')
                all_x.append(robot['x_aruco'])
                all_y.append(robot['y_aruco'])

        if all_x and all_y:
            self.ax.set_xlim(min(all_x) - 1, max(all_x) + 1)
            self.ax.set_ylim(min(all_y) - 1, max(all_y) + 1)

        self.ax.legend()
        plt.draw()
        plt.pause(0.1)

    def spin(self):
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.get_logger().info('Shutting down...')

def main(args=None):
    rclpy.init(args=args)
    node = RobotTrackerNode()

    try:
        node.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
