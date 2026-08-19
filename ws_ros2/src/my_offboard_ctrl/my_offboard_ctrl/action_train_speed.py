#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from std_msgs.msg import Float32MultiArray
import math

# Camera module
import cv2
# from .gzcam import GzCam # Décommenter si le module est présent
import threading

# def launch_cam_receiver():
#   cam = GzCam("/camera", (640,480))
#   while True:
#     img = cam.get_next_image()
#     cv2.imshow('pic-display', img)
#     cv2.waitKey(1)

class ActionsExecuter(Node):
    """Node for executing actions received from the RL model."""
    def __init__(self) -> None:
        super().__init__('actions_executer')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers PX4
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers PX4 & RL
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/drone/state', self.vehicle_local_position_callback, 10)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        self.subscription = self.create_subscription(Float32MultiArray,
            '/drone/actions',
            self.listener_callback,
            10)

        # Variables internes
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        
        # PX4 utilise le repère NED (Z vers le bas). -5.6 signifie 5.6m d'altitude.
        self.takeoff_height = -5.6 
        self.state = "TAKEOFF"
        
        # Cibles de vitesse envoyées par l'agent RL
        self.vx_target = 0.0
        self.vy_target = 0.0
        self.vz_target = 0.0
        
        # Timer à 50Hz (0.02s) pour maintenir le mode Offboard de PX4 stable
        self.timer = self.create_timer(0.02, self.timer_callback) 
        self.offboard_setpoint_counter = 0

    def vehicle_local_position_callback(self, vehicle_local_position):
        self.vehicle_local_position = msg.data

    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def publish_vehicle_command(self, command, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Envoie une consigne de POSITION (utilisé pour le décollage)"""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.velocity = [math.nan, math.nan, math.nan] # Obligatoire pour ignorer la vitesse
        msg.yaw = 0.0 
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_velocity_setpoint(self, vx: float, vy: float, vz: float):
        """Envoie une consigne de VITESSE (utilisé pour la mission RL)"""
        msg = TrajectorySetpoint()
        msg.position = [math.nan, math.nan, math.nan] # Obligatoire pour ignorer la position
        msg.velocity = [vx, vy, vz]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_offboard_control_mode(self, position=True, velocity=False):
        """Configure dynamiquement ce que le contrôleur interne de PX4 doit suivre."""
        msg = OffboardControlMode()
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def listener_callback(self, msg):
        """Reçoit les actions RL : [dvx, dvy, dvz]"""
        data = msg.data
        if len(data) >= 3:
            self.vx_target = float(data[0])
            self.vy_target = float(data[1])
            # INVERSION DU Z : RL considère que +Z c'est "monter", PX4 considère que -Z c'est "monter" (NED)
            self.vz_target = -float(data[2])

    def timer_callback(self):
        """Boucle de contrôle principale à 50Hz."""      

        # Attente d'avoir publié quelques setpoints avant de passer en offboard (sécurité PX4)
        if self.offboard_setpoint_counter < 10:
            self.publish_offboard_control_mode(position=True, velocity=False)
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            self.offboard_setpoint_counter += 1
            
        elif self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
            self.offboard_setpoint_counter += 1
            
        else:
            if self.state == "TAKEOFF":
                # Phase de décollage : On asservit en POSITION
                self.publish_offboard_control_mode(position=True, velocity=False)
                self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)

                # Vérification de l'altitude atteinte (z est négatif en NED)
                if abs(self.vehicle_local_position.z - self.takeoff_height) < 0.3:
                    self.get_logger().info("Takeoff successful, switching to RL VELOCITY CONTROL")
                    self.state = "MISSION"

            elif self.state == "MISSION":
                # Phase RL : On bascule l'Offboard pour qu'il suive des VITESSES
                self.publish_offboard_control_mode(position=False, velocity=True)
                self.publish_velocity_setpoint(self.vx_target, self.vy_target, self.vz_target)

def main(args=None) -> None:
    print('Starting actions executer node...')
    rclpy.init(args=args)
    node = ActionsExecuter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()