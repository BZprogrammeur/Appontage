import asyncio
import subprocess
import re
import math
from mavsdk import System
from stable_baselines3 import PPO

# Fonction utilitaire pour les Quaternions de MAVSDK
def quaternion_to_euler(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    # Roll
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # Pitch
    sinp = 2 * (w * y - z * x)
    pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi / 2, sinp)
    # Yaw
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

async def run_drone_telemetry():
    drone = System()
    # Connexion au port par défaut de la simulation PX4
    await drone.connect(system_address="udp://:14540")

    print("Connexion au drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone détecté !")
            break

    # On s'abonne à la position et à la vitesse en repère NED
    # (North, East, Down - le standard de PX4)
    print("--- Démarrage du flux de données ---")
    
    # On peut combiner les flux avec asyncio.gather ou simplement boucler sur le principal
    async for odom in drone.telemetry.position_velocity_ned():
        # Position
        pn, pe, pd = odom.position.north_m, odom.position.east_m, odom.position.down_m
        # Vitesse
        vn, ve, vd = odom.velocity.north_m_s, odom.velocity.east_m_s, odom.velocity.down_m_s
        
        # Récupération de l'attitude (pour roll, pitch, yaw)
        # Note : On récupère la dernière valeur connue pour ne pas bloquer
        async for attitude in drone.telemetry.attitude_quaternion():
            r, p, y = quaternion_to_euler(attitude)
            break 

        print(f"\rDRONE -> Pos: [{pn:.2f}, {pe:.2f}, {pd:.2f}] | Vel: [{vn:.2f}, {ve:.2f}]", end="")

async def get_drone_rotation_data():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Récupération des données de rotation...")

    # On s'abonne aux deux flux nécessaires
    async for attitude_q in drone.telemetry.attitude_quaternion():
        # 1. Angles d'attitude (Roll, Pitch, Yaw)
        roll, pitch, yaw = quaternion_to_euler(attitude_q)

        # 2. Vitesses angulaires (wx, wy, wz)
        # On récupère la valeur instantanée via un autre flux
        async for angular_vel in drone.telemetry.attitude_angular_velocity_body():
            wx = angular_vel.roll_rad_s
            wy = angular_vel.pitch_rad_s
            wz = angular_vel.yaw_rad_s
            break # On sort pour synchroniser avec l'attitude suivante

        print(f"\rAttitude: R:{roll:.2f} P:{pitch:.2f} Y:{yaw:.2f} | Rates: wx:{wx:.2f} wy:{wy:.2f} wz:{wz:.2f}", end="")
        
async def get_platform_full_pose(model_name="plateforme_hexapode"):
    try:
        cmd = ["gz", "topic", "-e", "-t", "/world/ocean/pose/info", "-n", "1"]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=1)
        
        if result.returncode != 0 or not result.stdout:
            return None

        blocks = result.stdout.split('pose {')
        for block in blocks:
            if f'name: "{model_name}"' in block:
                # 1. Extraction de la position (par défaut 0 si absent)
                pos_match = re.search(r"position \{(.*?)\}", block, re.DOTALL)
                px = py = pz = 0.0
                if pos_match:
                    p_data = pos_match.group(1)
                    px = float(re.search(r"x:\s+([\d.-]+)", p_data).group(1)) if "x:" in p_data else 0.0
                    py = float(re.search(r"y:\s+([\d.-]+)", p_data).group(1)) if "y:" in p_data else 0.0
                    pz = float(re.search(r"z:\s+([\d.-]+)", p_data).group(1)) if "z:" in p_data else 0.0

                # 2. Extraction de l'orientation (gestion des 0.0 absents)
                orient_match = re.search(r"orientation \{(.*?)\}", block, re.DOTALL)
                qx = qy = qz = 0.0
                qw = 1.0 # Le neutre d'un quaternion est w=1
                if orient_match:
                    o_data = orient_match.group(1)
                    qx = float(re.search(r"x:\s+([\d.-]+)", o_data).group(1)) if "x:" in o_data else 0.0
                    qy = float(re.search(r"y:\s+([\d.-]+)", o_data).group(1)) if "y:" in o_data else 0.0
                    qz = float(re.search(r"z:\s+([\d.-]+)", o_data).group(1)) if "z:" in o_data else 0.0
                    qw = float(re.search(r"w:\s+([\d.-]+)", o_data).group(1)) if "w:" in o_data else 1.0

                r, p, y = quaternion_to_euler(qx, qy, qz, qw)
                return {"pos": (px, py, pz), "euler": (r, p, y)}
    except Exception as e:
        print(f"Erreur : {e}")
    return None      
        
class DronePlatformBridge:
    def __init__(self):
        # Données Drone
        self.drone_pos = [0.0, 0.0, 0.0]  # N, E, D
        self.drone_vel = [0.0, 0.0, 0.0]  # VN, VE, VD
        self.drone_rpy = [0.0, 0.0, 0.0]  # Roll, Pitch, Yaw
        self.drone_rates = [0.0, 0.0, 0.0] # wx, wy, wz
        
        # Données Plateforme
        self.plateforme_pos = [0.0, 0.0, 0.0]
        self.plateforme_rpy = [0.0, 0.0, 0.0]

    # --- PARTIE DRONE (MAVSDK) ---
    async def update_drone_telemetry(self, drone):
        async for odom in drone.telemetry.position_velocity_ned():
            self.drone_pos = [odom.position.north_m, odom.position.east_m, odom.position.down_m]
            self.drone_vel = [odom.velocity.north_m_s, odom.velocity.east_m_s, odom.velocity.down_m_s]

    async def update_drone_attitude(self, drone):
        async for q in drone.telemetry.attitude_quaternion():
            # Utilise ta fonction quaternion_to_euler ici
            self.drone_rpy = list(quaternion_to_euler(q))

    async def update_drone_rates(self, drone):
        async for rates in drone.telemetry.attitude_angular_velocity_body():
            self.drone_rates = [rates.roll_rad_s, rates.pitch_rad_s, rates.yaw_rad_s]

    # --- PARTIE PLATEFORME (GAZEBO) ---
    async def update_platform_pose(self, model_name="plateforme_hexapode"):
        while True:
            # On utilise ta méthode subprocess robuste ici
            try:
                cmd = ["gz", "topic", "-e", "-t", "/world/ocean/pose/info", "-n", "1"]
                # shell=False pour la sécurité, timeout court pour ne pas bloquer
                proc = await asyncio.create_subprocess_exec(*cmd, stdout=asyncio.subprocess.PIPE)
                stdout, _ = await proc.communicate()
                output = stdout.decode()

                # Logique de parsing simplifiée (à adapter avec tes regex)
                if f'name: "{model_name}"' in output:
                    # Ici tu inseres ton code de parsing x,y,z et quaternions
                    # Simulation de mise à jour :
                    self.plateforme_pos = [5.0, 5.0, 0.7] # Valeurs extraites
            except:
                pass
            await asyncio.sleep(0.05) # Fréquence 20Hz pour la plateforme

    # --- BOUCLE PRINCIPALE IA ---
    async def run_ai_loop(self):
        print("Cerveau IA démarré. En attente de données...")
        await asyncio.sleep(2) # Attente de connexion
        
        while True:
            # 1. Calculer les entrées relatives pour ton IA
            # x_rel = drone_N - plateforme_X (Attention aux axes Gazebo vs NED !)
            x_rel = self.drone_pos[0] - self.plateforme_pos[0]
            y_rel = self.drone_pos[1] - self.plateforme_pos[1]
            z_rel = self.drone_pos[2] - self.plateforme_pos[2]

            # 2. Construire ton vecteur d'observation (12 valeurs)
            obs = [x_rel, y_rel, z_rel] + self.drone_vel + self.drone_rpy + self.drone_rates
            
            # 3. Inférence (Ici tu appelles ton modèle)
            action, _ = model.predict(obs, deterministic=True)
	
            print(f"\rREL_POS: {x_rel:.2f} {y_rel:.2f} {z_rel:.2f} | DRONE_ALT: {-self.drone_pos[2]:.2f}", end="")
            print(action)
            await asyncio.sleep(0.1) # Boucle IA à 10Hz

async def main():
    bridge = DronePlatformBridge()
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # Lancement de toutes les tâches en parallèle
    await asyncio.gather(
        bridge.update_drone_telemetry(drone),
        bridge.update_drone_attitude(drone),
        bridge.update_drone_rates(drone),
        bridge.update_platform_pose(),
        bridge.run_ai_loop()
    )

if __name__ == "__main__":
    model = PPO.load("ppo_drone_platform_pid_mobile_v6.zip")
    asyncio.run(main())
    
