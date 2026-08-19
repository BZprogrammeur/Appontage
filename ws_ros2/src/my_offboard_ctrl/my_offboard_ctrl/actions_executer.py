#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from std_msgs.msg import Float32MultiArray

# Camera module
import cv2
from .gzcam import GzCam
import threading

import sys
import termios
import tty

def launch_cam_receiver():
  cam = GzCam("/camera", (640,480))
  while True:
    img = cam.get_next_image()
    cv2.imshow('pic-display', img)
    cv2.waitKey(1)

class ActionsExecuter(Node):
    """Node for executing of actions received from the model."""
    def __init__(self) -> None:
        super().__init__('actions_executer')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.drone_state_suscriber = self.create_subscription(Float32MultiArray, '/drone/state', self.drone_state_callback, 10)
        self.subscription = self.create_subscription(Float32MultiArray,
            '/drone/actions',
            self.listener_callback,
            10)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.6
        self.state = "TAKEOFF"  # States: TAKEOFF, MISSION
        self.pose = [0.0] * 12  # [x, y, z, roll, pitch, yaw, vx, vy, vz, wr, wp, wy]

        self.x_target = 0.0
        self.y_target = 0.0
        self.z_target = 0.0
        
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz
        self.offboard_setpoint_counter = 0

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def drone_state_callback(self, msg):
        """Callback function for drone state topic subscriber."""
        self.pose = msg.data

    def arm(self):
            """Send an arm command to the vehicle."""
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.  # (0 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_velocity_setpoint(self, vx: float, vy: float, vz: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.velocity = [vx, vy, vz]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing velocity setpoints {[vx, vy, vz]}")

    def publish_offboard_control_mode(self):
        """Indique à PX4 quelles commandes il doit écouter."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def timer_callback(self):
        """Boucle de contrôle principale."""      
        self.publish_offboard_control_mode()

        if self.offboard_setpoint_counter < 10:
            self.offboard_setpoint_counter += 1
        elif self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
            self.offboard_setpoint_counter += 1
        else:
            if self.state == "TAKEOFF":
                self.get_logger().info("Waiting for takeoff...")
                self.get_logger().info(f"Current altitude: {self.pose[2]:.2f} m")
                self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)

                if abs(self.pose[2] - self.takeoff_height) < 0.3:
                    self.get_logger().info("Takeoff successful, switching to MISSION state")
                    self.state = "MISSION"

            elif self.state == "MISSION":
                self.publish_position_setpoint(self.x_target, self.y_target, self.z_target)
            self.offboard_setpoint_counter += 1

    def listener_callback(self, msg):
        data = msg.data
        if len(data) >= 3:
            self.x_target = self.pose[0] + data[0]
            self.y_target = self.pose[1] + data[1]
            self.z_target = self.pose[2] + data[2]

def main(args=None) -> None:
    """
    print('Starting camera...')
    cam_thread = threading.Thread(target=launch_cam_receiver)
    cam_thread.daemon = True
    cam_thread.start()
    """
    print('Starting actions executer node...')
    rclpy.init(args=args)

    node = ActionsExecuter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()