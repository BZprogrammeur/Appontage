import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import asyncio
import threading
import math
from mavsdk import System

class DroneTelemetryPublisher(Node):
    def __init__(self):
        super().__init__('drone_telemetry_publisher')
        # Topic de sortie pour le node de calcul
        self.publisher_ = self.create_publisher(Float32MultiArray, '/drone/state', 10)
        
        # Vecteur d'état : [x, y, z, r, p, y, vx, vy, vz, wr, wp, wy]
        self.state = [0.0] * 12
        
        # Thread séparé pour MAVSDK (Asynchrone)
        self.mavsdk_thread = threading.Thread(target=self.start_mavsdk, daemon=True)
        self.mavsdk_thread.start()
        
        # Timer ROS pour la publication (ex: 30Hz)
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)

    def start_mavsdk(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.run_mavsdk())

    async def run_mavsdk(self):
        drone = System()
        # On utilise udpin pour éviter les warnings et bypasser les soucis réseau
        await drone.connect(system_address="udpin://0.0.0.0:14540")
        
        print("Telemetry Node: En attente du drone...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                print("Telemetry Node: Drone connecté via MAVSDK")
                break

        # Lancement simultané des flux de données
        await asyncio.gather(
            self.get_position_velocity(drone),
            self.get_attitude_and_rates(drone)
        )

    async def get_position_velocity(self, drone):
        async for odom in drone.telemetry.position_velocity_ned():
            self.state[0:3] = [odom.position.north_m, odom.position.east_m, odom.position.down_m]
            self.state[6:9] = [odom.velocity.north_m_s, odom.velocity.east_m_s, odom.velocity.down_m_s]

    async def get_attitude_and_rates(self, drone):
        # Flux combiné Attitude + Rates
        async def update_rpy():
            async for q in drone.telemetry.attitude_quaternion():
                # Conversion Quaternion -> Euler
                w, x, y, z = q.w, q.x, q.y, q.z
                self.state[3] = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)) # Roll
                self.state[4] = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x)))) # Pitch
                self.state[5] = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)) # Yaw

        async def update_rates():
            async for r in drone.telemetry.attitude_angular_velocity_body():
                self.state[9:12] = [r.roll_rad_s, r.pitch_rad_s, r.yaw_rad_s]

        await asyncio.gather(update_rpy(), update_rates())

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = self.state
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DroneTelemetryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()