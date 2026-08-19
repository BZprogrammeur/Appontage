"""
Contrôle simplifié du mur de vent WindShape
Turbulence de Dryden + rafales 
"""

import windcontrol
import time
import math

# ==============================================================================
# PARAMÈTRES 
# ==============================================================================

WIND_MEAN_POWER   = 30.0   # puissance de base en % (0-100)
ALTITUDE          = 100.0  # altitude de vol simulée en m (influence Dryden)
AIRSPEED          = 10.0   # vitesse de l'air en m/s (influence Dryden)

TURBULENCE_INTENSITY = 1.5 # intensité de turbulence : légère=1, modérée=3, forte=6

GUST_POWER        = 20.0   # amplitude des rafales en % de puissance
GUST_FREQUENCY    = 0.15   # fréquence des rafales en Hz

UPDATE_RATE_HZ    = 20     # fréquence de mise à jour du mur (Hz)
DURATION_SEC      = 60     # durée totale de la simulation en secondes

seed = time.time_ns()

# ==============================================================================
# MODÈLE DE DRYDEN
# ==============================================================================

class DrydenFilter:
    def __init__(self, L, sigma, Va, order=1):
        self.L     = L
        self.sigma = sigma
        self.Va    = Va
        self.order = order
        self.x1    = 0.0
        self.x2    = 0.0

    def update(self, noise, dt):
        if self.order == 1:
            return self._order1(noise, dt)
        return self._order2(noise, dt)

    def _order1(self, noise, dt):
        tau     = self.L / self.Va
        a       = dt / (tau + dt)
        self.x1 = (1.0 - a) * self.x1 + \
                  a * self.sigma * math.sqrt(2.0 * self.L / (math.pi * self.Va)) * noise
        return self.x1

    def _order2(self, noise, dt):
        tau   = self.L / self.Va
        b     = self.sigma * math.sqrt(self.L / (math.pi * self.Va))
        a1    = 2.0 * dt / tau
        a2    = (dt / tau) ** 2
        denom = 1.0 + a1 + a2
        x_new = ((2.0 - a1) / denom) * self.x1 \
              - (1.0   / denom) * self.x2 \
              + (b * dt / denom) * noise
        self.x2 = self.x1
        self.x1 = x_new
        return self.x1


def make_dryden_filters(altitude, Va, turbulence_intensity):
    """Crée les 3 filtres Dryden (u, v, w) selon l'altitude (MIL-SPEC-1797A)."""
    L      = min(altitude / 2.0, 500.0)
    sigma_w = turbulence_intensity
    sigma_u = sigma_w / max(0.177 + 0.000823 * altitude, 0.01)

    return (
        DrydenFilter(L=L, sigma=sigma_u, Va=Va, order=1),  # u longitudinal
        DrydenFilter(L=L, sigma=sigma_u, Va=Va, order=2),  # v latéral
        DrydenFilter(L=L, sigma=sigma_w, Va=Va, order=2),  # w vertical
    )

def rand():
    global seed
    seed = (1664525 * seed + 1013904223) % (2**32)
    return seed / 2**32  # nombre entre 0 et 1

def compute_wind_power(dryden_u, dryden_v, dryden_w,
                       t, dt,
                       mean_power, gust_power, gust_freq):
    """
    Calcule la puissance totale à envoyer au mur.
    Retourne un float entre 0 et 100.
    """
    noise    = [rand() for _ in range(3)]
    turb_u   = dryden_u.update(noise[0], dt)
    turb_v   = dryden_v.update(noise[1], dt)
    turb_w   = dryden_w.update(noise[2], dt)

    # Turbulence totale → convertie en variation de puissance
    turb_norm   = math.sqrt(turb_u**2 + turb_v**2 + (turb_w * 0.3)**2)
    turb_power  = turb_norm * 5.0   # facteur d'échelle m/s → %

    # Rafale sinusoïdale
    gust        = gust_power * max(0.0, math.sin(2.0 * math.pi * gust_freq * t))

    power = mean_power + turb_power + gust
    return max(0.0, min(100.0, power))


# ==============================================================================
# PROGRAMME PRINCIPAL
# ==============================================================================

def main():
    dt = 1.0 / UPDATE_RATE_HZ

    print("Initialisation des filtres de Dryden...")
    dryden_u, dryden_v, dryden_w = make_dryden_filters(
        ALTITUDE, AIRSPEED, TURBULENCE_INTENSITY)

    print("Connexion au WindShaper...")
    ws = windcontrol.WindShaper(verbose=True)
    ws.startServerLink()
    ws.requestToken()
    time.sleep(1)
    ws.startPSUs()
    time.sleep(2)
    print("WindShaper prêt\n")

    print(f"Paramètres :")
    print(f"  Puissance de base  : {WIND_MEAN_POWER}%")
    print(f"  Altitude simulée   : {ALTITUDE} m")
    print(f"  Intensité Dryden   : {TURBULENCE_INTENSITY} m/s")
    print(f"  Rafales            : ±{GUST_POWER}% @ {GUST_FREQUENCY} Hz")
    print(f"  Durée              : {DURATION_SEC} s\n")
    print("─" * 50)

    t_start = time.time()
    t       = 0.0

    try:
        while t < DURATION_SEC:
            power = compute_wind_power(
                dryden_u, dryden_v, dryden_w,
                t, dt,
                WIND_MEAN_POWER, GUST_POWER, GUST_FREQUENCY)

            ws.setFanPower(power)

            print(f"  t={t:6.2f}s  puissance={power:5.1f}%", end='\r')

            time.sleep(dt)
            t += dt

    except KeyboardInterrupt:
        print("\nArrêt demandé")

    finally:
        print("\nExtinction du mur...")
        ws.setFanPower(0)
        ws.turnOffModules()
        ws.releaseToken()
        ws.stopServerLink()
        print("Terminé")


if __name__ == '__main__':
    main()