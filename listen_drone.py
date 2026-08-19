import asyncio
from mavsdk import System
import math

# --- UTILITAIRES ---
def quaternion_to_euler(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw

# --- LA CLASSE DE RÉCUPÉRATION ---
class DroneDataCollector:
    def __init__(self):
        self.drone = System()
        # Variables pour stocker l'état complet (ton vecteur d'observation)
        self.pos = [0, 0, 0]
        self.vel = [0, 0, 0]
        self.rpy = [0, 0, 0]
        self.rates = [0, 0, 0]

    async def start(self):
        # Connexion avec bypass du proxy pour éviter l'erreur 503
        await self.drone.connect(system_address="udpin://0.0.0.0:14540")
        print("Connexion au drone...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("Drone connecté !")
                break

        # Lancement des flux en parallèle
        await asyncio.gather(
            self.stream_telemetry(),
            self.stream_attitude(),
            self.display_loop()
        )

    async def stream_telemetry(self):
        """Flux pour la position et la vitesse (Translation)"""
        async for odom in self.drone.telemetry.position_velocity_ned():
            self.pos = [odom.position.north_m, odom.position.east_m, odom.position.down_m]
            self.vel = [odom.velocity.north_m_s, odom.velocity.east_m_s, odom.velocity.down_m_s]

    async def stream_attitude(self):
        """Flux pour l'orientation et les vitesses angulaires (Rotation)"""
        # On peut combiner les deux flux internes ici
        async def update_rates():
            async for v in self.drone.telemetry.attitude_angular_velocity_body():
                self.rates = [v.roll_rad_s, v.pitch_rad_s, v.yaw_rad_s]

        async def update_rpy():
            async for q in self.drone.telemetry.attitude_quaternion():
                self.rpy = list(quaternion_to_euler(q))

        await asyncio.gather(update_rates(), update_rpy())

    async def display_loop(self):
        """Boucle d'affichage (ou ton IA)"""
        while True:
            # Ici tu as accès à TOUTES les données en même temps
            print(f"\rPOS: {self.pos} | VEL: {self.vel} | RPY: {self.rpy} | RATES: {self.rates}", end="")
            await asyncio.sleep(0.1)

if __name__ == "__main__":
    collector = DroneDataCollector()
    try:
        asyncio.run(collector.start())
    except KeyboardInterrupt:
        print("\nArrêt du script.")