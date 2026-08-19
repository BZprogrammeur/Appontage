#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint ,VehicleCommand, VehicleStatus
from std_msgs.msg import Float32MultiArray
import numpy as np
import math

class ActionSpeed(Node):
    def __init__(self) -> None:
        super().__init__('action_executer_speed')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        # Subscriber au modèle IA
        self.subscription = self.create_subscription(Float32MultiArray, '/drone/actions', self.listener_callback, 10)
        self.drone_state_suscriber = self.create_subscription(Float32MultiArray, '/drone/state', self.drone_state_callback, 10)

        # Variables d'état
        self.offboard_setpoint_counter = 0
        self.target_vel = [0.0, 0.0, 0.0]
        self.takeoff_height = -4.0
        self.state = "TAKEOFF"  # States: TAKEOFF, MISSION
        self.pose = [0.0] * 12  


        # Timer à 50Hz (Le contrôle d'attitude demande une fréquence plus élevée que la position)
        self.timer = self.create_timer(0.02, self.timer_callback)

    def drone_state_callback(self, msg):
        """Callback function for drone state topic subscriber."""
        self.pose = msg.data
        
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.body_rate = False
        msg.attitude = False
        if self.state == "TAKEOFF":
            msg.position = True
            msg.velocity = False
        else: # MISSION_IA
            msg.position = False
            msg.velocity = True
        
        self.offboard_control_mode_publisher.publish(msg)

    def publish_velocity_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # On force PX4 à ignorer le contrôleur de position
        msg.position = [float('nan'), float('nan'), float('nan')]
        
        # On injecte tes commandes de vitesse en m/s
        msg.velocity = [float(self.target_vel[0]), float(self.target_vel[1]), float(self.target_vel[2])]
        msg.yaw = 0.0 
        
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.  # (0 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")    

    def listener_callback(self, msg):
        """Reçoit les actions [dvx, dvy, dvz] calculées par le modèle IA."""
        if len(msg.data) == 3:
            ia_vx = float(msg.data[0])
            ia_vy = float(msg.data[1])
            ia_vz = float(msg.data[2])

            # 1. Extraction du Yaw actuel du drone (nécessaire pour projeter la vitesse)
            current_yaw = self.pose[5] 

            # 2. PROJECTION :
            # il faut les projeter dans le repère GLOBAL (NED) de PX4 :
            cos_y = math.cos(current_yaw)
            sin_y = math.sin(current_yaw)
            
            px4_vx = ia_vx * cos_y - ia_vy * sin_y
            px4_vy = ia_vx * sin_y + ia_vy * cos_y

            # 3. INVERSION DE L'AXE Z : PX4 utilise le repère NED, donc +Z est vers le bas. Si ton IA considère +Z comme "monter", il faut inverser :
            px4_vz = -ia_vz 

            # On applique la consigne finale propre
            self.target_vel = [px4_vx, px4_vy, px4_vz]
            
            # Envoi immédiat à PX4 si on est en l'air
            if self.state == "MISSION":
                self.publish_velocity_setpoint()

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
                self.publish_velocity_setpoint()
            self.offboard_setpoint_counter += 1         

    # --- Utilitaires de commande (Arm, Offboard) ---
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def publish_vehicle_command(self, command, **kwargs):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = kwargs.get("param1", 0.0)
        msg.param2 = kwargs.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ActionSpeed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()