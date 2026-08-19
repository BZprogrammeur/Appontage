import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from stable_baselines3 import PPO
import numpy as np

state = "SPEED"  # States: POSITION, SPEED, ACCELERATION

if state == "POSITION":
    model_path = "/home/PX4-ROS2-Gazebo-Drone-Simulation-Template/ppo_drone_platform_pid_mobile_v6"
elif state == "SPEED":
    model_path = "/home/PX4-ROS2-Gazebo-Drone-Simulation-Template/ppo_drone_speed_mode3_tb.zip"
elif state == "ACCELERATION":
    model_path = "/home/PX4-ROS2-Gazebo-Drone-Simulation-Template/ppo_drone_platform_pid_mobile_v6_acceleration"

model = PPO.load(model_path, device='cpu')

plateforme_pos = np.array([3, 3, 0.5 , 0, 0, 0, 0, 0, 0, 0, 0, 0]) 
wind_force = np.array([0, 0, 0])

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        # Création du Subscriber qui écoute le topic /drone/state
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/drone/state',
            self.listener_callback,
            10)
        self.subscription  # pour éviter que la variable ne soit supprimée par le garbage collector

        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            '/drone/actions', 
            10)
        self.get_logger().info('Node Drone Controller démarré, en attente de données...')

    def listener_callback(self, msg):
        data = msg.data
        
        if len(data) == 12:
            # Données du drone issues du topic /drone/state
            pos_x, pos_y, pos_z = data[0], data[1], data[2]
            roll, pitch, yaw = data[3], data[4], data[5]
            vx, vy, vz = data[6], data[7], data[8]
            wr, wp, wy = data[9], data[10], data[11]

            # Coordonnées réelles de ta plateforme fixe dans Gazebo
            p_x, p_y, p_z = 5.0, 5.0, 0.5
            
            # Reconstruction fidèle de l'observation Gymnasium à 21 dimensions
            observation = np.array([
                pos_x - p_x, pos_y - p_y, pos_z - p_z,  # 0,1,2 : Position relative
                vx, vy, vz,                             # 3,4,5 : Vitesse drone
                roll, pitch, yaw,                       # 6,7,8 : Attitude drone
                wr, wp, wy,                             # 9,10,11 : Vitesses angulaires drone
                0.0, 0.0, 0.0,                          # 12,13,14 : Vent (0 si désactivé)
                0.0, 0.0, 0.0,                          # 15,16,17 : VRAIE VITESSE plateforme (Fixe = 0)
                0.0, 0.0, 0.0                           # 18,19,20 : VRAI RPY plateforme (À plat = 0)
            ], dtype=np.float32)

            # Inférence
            action, _state = model.predict(observation, deterministic=True)

            # Publication vers action_speed
            action_msg = Float32MultiArray()
            action_msg.data = action.tolist()
            self.publisher_.publish(action_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()