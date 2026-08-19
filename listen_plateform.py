import subprocess
import time
import math
import re

def quaternion_to_euler(x, y, z, w):
    """Convertit un quaternion en Roll, Pitch, Yaw (radians)."""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw

def get_platform_full_pose(model_name="plateforme_hexapode"):
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
                    qw = float(re.search(r"w:\s+([\d.-]+)", o_data).group(1)) if "w:" in o_data 				else 1.0

                r, p, y = quaternion_to_euler(qx, qy, qz, qw)
                return {"pos": (px, py, pz), "euler": (r, p, y)}
    except Exception as e:
        print(f"Erreur : {e}")
    return None

# --- BOUCLE ---
print(f"Écoute de la plateforme...")
while True:
    res = get_platform_full_pose()
    if res:
        p, e = res["pos"], res["euler"]
        print(f"\rX:{p[0]:.2f} Y:{p[1]:.2f} Z:{p[2]:.2f} | R:{e[0]:.2f} P:{e[1]:.2f} Y:{e[2]:.2f}", end="")
    else:
        print("\rEn attente du modèle...", end="")
    time.sleep(0.1)
