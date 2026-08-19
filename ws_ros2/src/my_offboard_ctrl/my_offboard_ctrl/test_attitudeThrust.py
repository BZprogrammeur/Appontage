#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class TestPIDBodyRatesNode(Node):
    def __init__(self):
        super().__init__('test_pid_body_rates')

        # Subscriber pour connaître l'état complet du drone
        # Attend 12 valeurs : [X, Y, Z, Vx, Vy, Vz, Roll, Pitch, Yaw, R_rate, P_rate, Y_rate]
        self.state_sub = self.create_subscription(
            Float32MultiArray, '/drone/state', self.state_callback, 10)
            
        # Publisher vers ton ActionExecuter (qui gère désormais le VehicleRatesSetpoint)
        self.action_pub = self.create_publisher(
            Float32MultiArray, '/drone/actions_attitudeThrust', 50)

        # --- CONFIGURATION CIBLES ---
        # Si ton simulateur utilise le repère standard NED (Z négatif en l'air), mets -2.0. 
        # Si ton topic /drone/state a inversé Z pour qu'il soit positif en l'air, laisse 2.0.
        self.target_x, self.target_y, self.target_z = 0.0, 0.0, -2.0 
        self.target_yaw = 0.0

        # --- ÉTAT ACTUEL ---
        self.current_pos = [0.0, 0.0, 0.0]
        self.current_vel = [0.0, 0.0, 0.0]
        self.current_rot = [0.0, 0.0, 0.0] # Roll, Pitch, Yaw actuels (en radians)

        # --- GAINS DU CONTROLEUR EN CASCADE ---
        # 1. Boucle Extérieure : Position -> Vitesse désirée
        self.kp_pos = 0.7
        
        # 2. Boucle Intermédiaire : Vitesse -> Angles désirés (Roll/Pitch target)
        self.kp_vel = 0.15
        self.kd_vel = 0.05
        
        # 3. Boucle Intérieure : Angles -> Vitesse angulaire (Body Rates rad/s)
        self.kp_angle = 4.0 # Un gain fort car on veut annuler l'erreur d'angle très vite

        # 4. Boucle Verticale : Altitude -> Thrust
        self.kp_z = 0.25
        self.ki_z = 0.02
        self.kd_z = 0.15
        self.hover_thrust = 0.58  # Ajusté proche du point de stationnaire Gazebo
        
        self.integral_z = 0.0
        self.last_error_z = 0.0

        # Boucle de contrôle à 50Hz
        self.timer = self.create_timer(0.02, self.control_loop)

    def state_callback(self, msg):
        """
        Découpage du vecteur d'état reçu de l'environnement.
        Assure-toi que les index correspondent bien à la structure de ton vecteur !
        """
        if len(msg.data) >= 9:
            self.current_pos = msg.data[0:3]  # X, Y, Z
            self.current_vel = msg.data[3:6]  # Vx, Vy, Vz
            self.current_rot = msg.data[6:9]  # Roll, Pitch, Yaw (en radians)

    def control_loop(self):
        # -----------------------------------------------------------
        # 1. PID VERTICAL (Altitude -> Thrust)
        # -----------------------------------------------------------
        error_z = self.target_z - self.current_pos[2]
        
        self.integral_z += error_z * 0.02
        self.integral_z = np.clip(self.integral_z, -0.3, 0.3) # Anti-windup
        
        derivative_z = (error_z - self.last_error_z) / 0.02
        self.last_error_z = error_z

        thrust_delta = (error_z * self.kp_z) + (self.integral_z * self.ki_z) + (derivative_z * self.kd_z)
        thrust = self.hover_thrust + thrust_delta
        thrust = np.clip(thrust, 0.0, 1.0)

        # -----------------------------------------------------------
        # 2. BOUCLE HORIZONTALE EN CASCADE (Position -> Vitesses -> Angles)
        # -----------------------------------------------------------
        # Calcul des vitesses linéaires cibles pour rejoindre la position
        vel_target_x = (self.target_x - self.current_pos[0]) * self.kp_pos
        vel_target_y = (self.target_y - self.current_pos[1]) * self.kp_pos
        
        # Saturation des vitesses max de sécurité (ex: 2 m/s)
        vel_target_x = np.clip(vel_target_x, -2.0, 2.0)
        vel_target_y = np.clip(vel_target_y, -2.0, 2.0)

        # Erreurs de vitesse linéaire
        error_vel_x = vel_target_x - self.current_vel[0]
        error_vel_y = vel_target_y - self.current_vel[1]

        # Calcul des angles cibles (Roll et Pitch désirés)
        # En NED : avancer (X+) demande de pencher le nez vers le bas (Pitch -)
        # En NED : aller à droite (Y+) demande de pencher à droite (Roll +)
        pitch_target = -(error_vel_x * self.kp_vel)
        roll_target  =  (error_vel_y * self.kp_vel)

        # Saturation des angles max (Max 12 degrés pour rester stable au début)
        max_angle = 0.21  # radians
        pitch_target = np.clip(pitch_target, -max_angle, max_angle)
        roll_target  = np.clip(roll_target, -max_angle, max_angle)

        # -----------------------------------------------------------
        # 3. BOUCLE DE RATES (Angles désirés -> Body Rates rad/s)
        # -----------------------------------------------------------
        # On calcule le taux de rotation nécessaire pour atteindre l'angle désiré
        roll_rate  = (roll_target  - self.current_rot[0]) * self.kp_angle
        pitch_rate = (pitch_target - self.current_rot[1]) * self.kp_angle
        yaw_rate   = (self.target_yaw - self.current_rot[2]) * self.kp_angle

        # Saturation des rates (Max 1.0 rad/s (~57°/s) pour éviter des mouvements trop brusques)
        max_rate = 1.0
        roll_rate  = np.clip(roll_rate, -max_rate, max_rate)
        pitch_rate = np.clip(pitch_rate, -max_rate, max_rate)
        yaw_rate   = np.clip(yaw_rate, -max_rate, max_rate)

        # -----------------------------------------------------------
        # 4. ENVOI DE L'ACTION SYNCHRONISÉE
        # -----------------------------------------------------------
        msg = Float32MultiArray()
        # Le format attendu par ton nouveau node : [roll_rate, pitch_rate, yaw_rate, thrust]
        msg.data = [float(roll_rate), float(pitch_rate), float(yaw_rate), float(thrust)]
        self.action_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestPIDBodyRatesNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()