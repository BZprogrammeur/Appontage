#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint ,VehicleCommand, VehicleStatus, VehicleRatesSetpoint, VehicleThrustSetpoint
from std_msgs.msg import Float32MultiArray
import numpy as np
import math

class ActionExecuterAttitudeThrust(Node):
    def __init__(self) -> None:
        super().__init__('action_executer_attitudeThrust')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, 
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.attitude_setpoint_publisher = self.create_publisher(VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_profile)
        self.thrust_setpoint_publisher = self.create_publisher(VehicleThrustSetpoint, '/fmu/in/vehicle_thrust_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        # Subscriber au modèle IA
        self.subscription = self.create_subscription(Float32MultiArray, '/drone/actions_attitudeThrust', self.listener_callback, 10)
        self.drone_state_suscriber = self.create_subscription(Float32MultiArray, '/drone/state', self.drone_state_callback, 10)

        # Variables d'état
        self.offboard_setpoint_counter = 0
        self.action_attitude = [0.0, 0.0, 0.0] # Roll, Pitch, Yaw
        self.action_thrust = 0.0                # Thrust entre 0 et 1
        self.takeoff_height = -4.0
        self.state = "TAKEOFF"  
        self.pose = [0.0] * 12  


        # Timer à 50Hz (Le contrôle d'attitude demande une fréquence plus élevée que la position)
        self.timer = self.create_timer(0.02, self.timer_callback)

    def euler_to_quaternion(self, r, p, y):
        """Convertit Roll, Pitch, Yaw en Quaternion [w, x, y, z]"""
        cy = math.cos(y * 0.5)
        sy = math.sin(y * 0.5)
        cp = math.cos(p * 0.5)
        sp = math.sin(p * 0.5)
        cr = math.cos(r * 0.5)
        sr = math.sin(r * 0.5)

        q = [0.0] * 4
        q[0] = cr * cp * cy + sr * sp * sy # w
        q[1] = sr * cp * cy - cr * sp * sy # x
        q[2] = cr * sp * cy + sr * cp * sy # y
        q[3] = cr * cp * sy - sr * sp * cy # z
        return q

    def drone_state_callback(self, msg):
        """Callback function for drone state topic subscriber."""
        self.pose = msg.data
        
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # On initialise TOUT explicitement à chaque appel pour éviter les résidus
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        
        # Optionnel selon ta version de px4_msgs, décommenter si nécessaire :
        # msg.actuator = False 

        if self.state == "TAKEOFF":
            msg.position = True
        else: # MISSION
            msg.body_rate = True
        
        self.offboard_control_mode_publisher.publish(msg)

    def publish_attitude_setpoint(self):
        msg = VehicleRatesSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Quaternion depuis roll/pitch/yaw
        #msg.q_d = self.euler_to_quaternion(*self.action_attitude)
        
        # thrust_body[2] DOIT être négatif pour monter en body frame NED
        thrust_z = -1.0 * np.clip(self.action_thrust, 0.0, 1.0)
        msg.thrust_body = [0.0, 0.0, thrust_z]  
        msg.roll = 0.0
        msg.pitch = 0.0
        msg.yaw = 0.01
        msg.reset_integral = False

        self.attitude_setpoint_publisher.publish(msg)  
        self.get_logger().info(f"Publishing thrust setpoint {msg.thrust_body[2]:.2f}")     

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.  # (0 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")    

    def listener_callback(self, msg):
        """
        On attend 4 valeurs : [roll, pitch, yaw, thrust]
        Les angles doivent être en radians.
        """
        if len(msg.data) == 4:
            self.action_attitude = msg.data[0:3]
            self.action_thrust = msg.data[3]

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
                self.publish_attitude_setpoint()
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
    node = ActionExecuterAttitudeThrust()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()