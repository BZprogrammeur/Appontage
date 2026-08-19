import os
import time
import math

# Configuration
TOPIC = "/model/plateforme_hexapode/cmd_vel"
AMPLITUDE_PITCH = 0.3  # Radian/s
AMPLITUDE_Z = 0      # m/s
FREQUENCE = 0.2        # Hz (vitesse de l'oscillation)

print(f"Démarrage de la trajectoire sur {TOPIC}...")

start_time = time.time()

try:
    while True:
        # Temps écoulé
        t = time.time() - start_time
        
        # Calcul des vitesses (Fonction Sinus)
        # v = Amplitude * sin(2 * pi * f * t)
        val_pitch = AMPLITUDE_PITCH * math.sin(2 * math.pi * FREQUENCE * t)
        val_z = AMPLITUDE_Z * math.sin(2 * math.pi * FREQUENCE * t)

        # Construction de la commande shell
        cmd = (
            f'gz topic -t "{TOPIC}" -m gz.msgs.Twist -p '
            f'"linear: {{z: {val_z}}}, angular: {{y: {val_pitch}}}"'
        )
        
        # Envoi à Gazebo
        os.system(cmd)
        
        # Fréquence de rafraîchissement du script (20Hz)
        time.sleep(0.05)

except KeyboardInterrupt:
    # Arrêt propre : on remet les vitesses à zéro
    print("\nArrêt du mouvement...")
    os.system(f'gz topic -t "{TOPIC}" -m gz.msgs.Twist -p "linear: {{z: 0}}, angular: {{y: 0}}"')
