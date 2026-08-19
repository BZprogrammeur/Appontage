"""
Bridge ROS 2 → WindShaper
Reproduit le vent simulé (plugin Gazebo) sur le mur de vent physique
"""

import windcontrol
import time
import threading
import math
import numpy as np

# ── ROS 2 ─────────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from wind_msgs.msg import WindCmd

# ==============================================================================
# MODÈLE DE DRYDEN (identique au plugin C++)
# ==============================================================================

class DrydenFilter:
    """Filtre de turbulence de Dryden — ordre 1 (longitudinal) ou 2 (latéral)."""

    def __init__(self, L=200.0, sigma=1.0, Va=10.0):
        self.L     = L
        self.sigma = sigma
        self.Va    = Va
        self.x1    = 0.0
        self.x2    = 0.0

    def update_order1(self, noise, dt):
        """Axe longitudinal (X)."""
        tau     = self.L / self.Va
        a       = dt / (tau + dt)
        self.x1 = (1.0 - a) * self.x1 + a * self.sigma * math.sqrt(
                    2.0 * self.L / (math.pi * self.Va)) * noise
        return self.x1

    def update_order2(self, noise, dt):
        """Axes latéraux (Y, Z)."""
        tau  = self.L / self.Va
        b    = self.sigma * math.sqrt(self.L / (math.pi * self.Va))
        a1   = 2.0 * dt / tau
        a2   = (dt / tau) ** 2
        denom = 1.0 + a1 + a2
        x_new = ((2.0 - a1) / denom) * self.x1 \
              - (1.0 / denom) * self.x2 \
              + (b * dt / denom) * noise
        self.x2 = self.x1
        self.x1 = x_new
        return self.x1


class WindModel:
    """Calcule le vecteur de vent total à partir des paramètres WindCmd."""

    def __init__(self, altitude=100.0):
        # Paramètres Dryden selon altitude
        L_horiz = min(altitude / 2.0, 500.0)
        L_vert  = L_horiz
        sigma_w = 1.0  # sera mis à jour depuis WindCmd
        sigma_u = sigma_w / max(0.177 + 0.000823 * altitude, 0.01)

        self.dryden_u = DrydenFilter(L=L_horiz, sigma=sigma_u)
        self.dryden_v = DrydenFilter(L=L_horiz, sigma=sigma_u)
        self.dryden_w = DrydenFilter(L=L_vert,  sigma=sigma_w)

        # Paramètres courants (mis à jour depuis /wind/current)
        self.force_mean       = [0.0, 0.0, 0.0]
        self.force_variance   = 0.0
        self.gust_magnitude   = 0.0
        self.gust_frequency   = 0.1
        self.sim_time         = 0.0
        self.lock             = threading.Lock()

    def update_params(self, msg: WindCmd):
        with self.lock:
            self.force_mean     = [msg.force_mean.x,
                                   msg.force_mean.y,
                                   msg.force_mean.z]
            self.force_variance = msg.force_variance
            self.gust_magnitude = msg.gust_force_magnitude
            self.gust_frequency = msg.gust_frequency
            # Mettre à jour sigma des filtres Dryden
            self.dryden_u.sigma = math.sqrt(self.force_variance) if self.force_variance > 0 else 0.0
            self.dryden_v.sigma = self.dryden_u.sigma
            self.dryden_w.sigma = self.dryden_u.sigma * 0.5

    def compute(self, dt: float) -> tuple:
        """Retourne (vx, vy, vz) en m/s."""
        with self.lock:
            noise = np.random.randn(3)
            turb_u = self.dryden_u.update_order1(noise[0], dt)
            turb_v = self.dryden_v.update_order2(noise[1], dt)
            turb_w = self.dryden_w.update_order2(noise[2], dt)

            self.sim_time += dt
            gust = self.gust_magnitude * math.sin(
                2.0 * math.pi * self.gust_frequency * self.sim_time)

            vx = self.force_mean[0] + turb_u + gust
            vy = self.force_mean[1] + turb_v
            vz = self.force_mean[2] + turb_w * 0.3

            return (vx, vy, vz)


# ==============================================================================
# NŒUD ROS 2 — écoute /wind/current
# ==============================================================================

class WindBridgeNode(Node):
    def __init__(self, wind_model: WindModel):
        super().__init__('windshaper_bridge')
        self.wind_model = wind_model
        self.create_subscription(
            WindCmd,
            '/wind/current',   # ce que publie le plugin Gazebo
            self.on_wind,
            10)
        self.get_logger().info("Abonné à /wind/current")

    def on_wind(self, msg: WindCmd):
        self.wind_model.update_params(msg)


# ==============================================================================
# WINDSHAPER BRIDGE
# ==============================================================================

def norm_to_power(v: float, v_max: float = 15.0) -> int:
    """Convertit une vitesse de vent (m/s) en puissance fan (0-100%)."""
    power = int(abs(v) / v_max * 100)
    return max(0, min(100, power))


def run_windshaper(wind_model: WindModel, duration: float = 0.0):
    """
    Boucle principale WindShaper.
    duration=0 → tourne indéfiniment jusqu'à Ctrl+C
    """
    print("Connexion au WindShaper...")
    ws = windcontrol.WindShaper(verbose=False)
    ws.startServerLink()
    ws.requestToken()
    time.sleep(1)
    ws.startPSUs()
    time.sleep(2)

    fan_units = ws.getFanUnits()
    if fan_units == -1:
        print("Impossible de récupérer les fan units")
        ws.stopServerLink()
        return

    n_rows = 8
    n_cols = 8
    print(f"Mur de vent : {n_rows} rangées × {n_cols} colonnes")

    dt      = 0.05   # 20 Hz — fréquence de mise à jour du mur
    t_start = time.time()

    print("Bridge actif — Ctrl+C pour arrêter")

    try:
        while True:
            t_now = time.time()
            if duration > 0 and (t_now - t_start) > duration:
                break

            vx, vy, vz = wind_model.compute(dt)

            # Puissance globale basée sur la norme du vecteur vent
            v_norm  = math.sqrt(vx**2 + vy**2)
            power   = norm_to_power(v_norm)

            setFanPower(power, fan_layer = 1) 
            setFanPower(0.7*power, fan_layer = 2) 

            print(f"Vent: vx={vx:+.2f} vy={vy:+.2f} vz={vz:+.2f} m/s "
                  f"→ puissance={power}%", end='\r')

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n Arrêt demandé")

    finally:
        print("Extinction du mur de vent...")
        for j in range(n_rows):
            for k in range(n_cols):
                fan_units[0][j][k].setFanPower(0)
        ws.turnOffModules()
        ws.releaseToken()
        ws.stopServerLink()


# ==============================================================================
# MAIN
# ==============================================================================

def main():
    wind_model = WindModel(altitude=100.0)

    # Thread ROS 2
    rclpy.init()
    node = WindBridgeNode(wind_model)
    ros_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Boucle WindShaper (thread principal)
    run_windshaper(wind_model, duration=0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()