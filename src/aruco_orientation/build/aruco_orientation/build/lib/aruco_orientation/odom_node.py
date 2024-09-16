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

# Constants
MARKER_ID = 3  # ArUco marker ID corresponding to the robot (for quiver plot)
ALPHA = 1.0  # Weight for the complementary filter (trust ArUco more)

class RobotTrackerNode(Node):
    def __init__(self):
        super().__init__('robot_tracker_node')

        # Variables for position (from ArUco) and orientation (from /odom)
        self.x = 0.0
        self.y = 0.0
        self.x_secondary = 0.0
        self.y_secondary = 0.0  # X, Y coordinates for secondary ArUco markers
        self.yaw = 0.0
        self.yaw_odom = 0.0  # Yaw from odometry
        self.yaw_aruco = 0.0  # Yaw from ArUco marker
        
        # Variables for marker ID 1
        self.x_marker1 = 0.0
        self.y_marker1 = 0.0
        self.yaw_marker1 = 0.0

        # Subscribe to /odom to receive robot orientation (yaw)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Initialize the plot for visualization
        self.init_plot()

        # Initialize ZED Camera
        self.zed = self.initialize_zed_camera()

        # Publishers for x, y, and yaw (orientation) for marker ID 3
        self.publisher_x = self.create_publisher(Float32, 'aruco_x', 10)
        self.publisher_y = self.create_publisher(Float32, 'aruco_y', 10)
        self.publisher_yaw = self.create_publisher(Float32, 'aruco_yaw', 10)

        # Publishers for x, y, and yaw (orientation) for marker ID 1
        self.publisher_x_marker1 = self.create_publisher(Float32, 'aruco1_x', 10)
        self.publisher_y_marker1 = self.create_publisher(Float32, 'aruco1_y', 10)
        self.publisher_yaw_marker1 = self.create_publisher(Float32, 'aruco1_yaw', 10)

        # Example camera matrix and distortion coefficients (replace with your calibrated values)
        self.camera_matrix = np.array([[1000, 0, 640], [0, 1000, 360], [0, 0, 1]], dtype=float)
        self.dist_coeffs = np.zeros((5, 1))  # Assuming no distortion

        # Prepare ZED Mat objects
        resolution = sl.Resolution(1280, 720)
        self.image_zed = sl.Mat(resolution.width, resolution.height, sl.MAT_TYPE.U8_C4)
        self.depth_map = sl.Mat(resolution.width, resolution.height, sl.MAT_TYPE.F32_C1)

        # Dictionary and parameters for ArUco detection
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # Spin the node
        self.timer = self.create_timer(0.1, self.detect_and_update)

    def init_plot(self):
        """Initialize the plot."""
        self.fig, self.ax = plt.subplots()
        self.ax.set_title('Robot Position and Orientation')
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        plt.ion()  # Enable interactive mode
        plt.show()

    def initialize_zed_camera(self):
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use ultra depth mode for higher accuracy
        init_params.coordinate_units = sl.UNIT.METER  # Set units to meters
        init_params.camera_resolution = sl.RESOLUTION.HD2K
        
        zed = sl.Camera()
        if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            print("Unable to open ZED Camera")
            zed.close()
            exit(1)
        
        # Enable positional tracking
        positional_tracking_parameters = sl.PositionalTrackingParameters()
        positional_tracking_parameters.set_as_static = True  # Set to true if the camera is static
        zed.enable_positional_tracking(positional_tracking_parameters)
        
        return zed

    def complementary_filter(self, yaw_aruco, yaw_odom, alpha=ALPHA):
        return alpha * yaw_aruco + (1 - alpha) * yaw_odom

    def get_yaw_from_aruco(self, rvec):
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        return yaw

    def detect_and_update(self):
        """Detect ArUco markers and update the robot's position and orientation."""
        rt_param = sl.RuntimeParameters()
        rt_param.measure3D_reference_frame = sl.REFERENCE_FRAME.WORLD  # Use WORLD reference frame

        # Grab the image from ZED
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT)
            image_ocv = self.image_zed.get_data()[:, :, :3]  # Extract RGB
            image_ocv_bgr = cv2.cvtColor(image_ocv, cv2.COLOR_RGB2BGR)

            # Detect ArUco markers
            corners, ids, _ = cv2.aruco.detectMarkers(image_ocv_bgr, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None:
                for i, marker_id in enumerate(ids):
                    marker_size = 0.05  # Marker size in meters
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, self.camera_matrix, self.dist_coeffs)

                    if marker_id == MARKER_ID:
                        # Process marker ID 3 (existing logic)
                        self.x, self.y = tvecs[i][0][0], -tvecs[i][0][1]  # Negate both x and y coordinates
                        self.yaw_aruco = -self.get_yaw_from_aruco(rvecs[i])
                        self.yaw = self.complementary_filter(self.yaw_aruco, self.yaw_odom)

                        # Publish x, y, and yaw for marker ID 3
                        self.publisher_x.publish(Float32(data=self.x))
                        self.publisher_y.publish(Float32(data=self.y))
                        self.publisher_yaw.publish(Float32(data=self.yaw))

                    elif marker_id == 1:
                        # Process marker ID 1 (new logic)
                        self.x_marker1, self.y_marker1 = tvecs[i][0][0], -tvecs[i][0][1]
                        self.yaw_marker1 = -self.get_yaw_from_aruco(rvecs[i])

                        # Publish x, y, and yaw for marker ID 1
                        self.publisher_x_marker1.publish(Float32(data=self.x_marker1))
                        self.publisher_y_marker1.publish(Float32(data=self.y_marker1))
                        self.publisher_yaw_marker1.publish(Float32(data=self.yaw_marker1))

                    else:
                        # Process other markers (e.g., secondary marker)
                        self.x_secondary, self.y_secondary = tvecs[i][0][0], -tvecs[i][0][1]

                # Update the plot with the new position and orientation
                self.update_plot()

    def odom_callback(self, msg):
        """Callback for the /odom topic to extract the robot's yaw."""
        orientation = msg.pose.pose.orientation
        self.yaw_odom = math.atan2(2 * (orientation.w * orientation.z + orientation.x * orientation.y),
                              1 - 2 * (orientation.y ** 2 + orientation.z ** 2))

    def update_plot(self):
        """Update the plot with the robot's position and orientation."""
        u = math.cos(self.yaw)  # X component of direction vector (from yaw)
        v = math.sin(self.yaw)  # Y component of direction vector (from yaw)

        # Clear the plot and refresh
        self.ax.clear()
        self.ax.set_title('Robot Position and Orientation')
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.grid(True)

        # Plot the robot's position and orientation for MARKER_ID (3)
        self.ax.quiver(self.x, self.y, u, v, angles='xy', scale_units='xy', scale=1, color='r', label="Orientation")

        # Plot marker ID 1 position and orientation
        u_marker1 = math.cos(self.yaw_marker1)
        v_marker1 = math.sin(self.yaw_marker1)
        self.ax.quiver(self.x_marker1, self.y_marker1, u_marker1, v_marker1, angles='xy', scale_units='xy', scale=1, color='b', label="Marker 1 Orientation")

        # Plot the secondary ArUco marker's x, y position (without orientation)
        self.ax.scatter(self.x_secondary, self.y_secondary, color='g', label="Secondary Marker")

        # Set the axis limits around the robot's position
        all_x = [self.x, self.x_marker1, self.x_secondary]
        all_y = [self.y, self.y_marker1, self.y_secondary]
        self.ax.set_xlim(min(all_x) - 1, max(all_x) + 1)
        self.ax.set_ylim(min(all_y) - 1, max(all_y) + 1)

        # Draw the plot
        plt.draw()
        plt.pause(0.1)  # Adjust the pause duration as needed

    def spin(self):
        """Main loop to run the node and plot updates."""
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