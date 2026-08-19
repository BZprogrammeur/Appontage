import time
import math
import numpy as np
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node

# ==========================================
# 1. PARAMÈTRES DU MODÈLE HYDRODYNAMIQUE
# ==========================================
M_tot = np.eye(6) * 10000.0
M_tot[0, 0] = 500000.0  # Masse totale en X (cavalement)
M_tot[1, 1] = 500000.0  # Masse totale en Y (embardée)
M_tot[5, 5] = 200000.0  # Inertie en lacet (Rz)
C = np.eye(6) * 500.0

K = np.zeros(6)
K[2] = 2000.0  # Raideur pilonnement (z)
K[3] = 5000.0   # Raideur roulis
K[4] = 5000.0   # Raideur tangage

def force_houle(t):
    """ Calcule l'effort de la houle à l'instant t """
    F = np.zeros(6)
    omega = 1.2  # Pulsation
    F[2] = 1000.0 * np.cos(omega * t)  # Force en Z (pilonnement)
    # Optionnel : tu peux ajouter un moment en Tangage (y) induit par la houle ici :
    F[4] = 1000.0 * np.sin(omega * t) 
    return F

# Initialisation des états : [x, y, z, rx, ry, rz]
S = np.zeros(6)       # Positions / Angles
dSdt = np.zeros(6)    # Vitesses linéaires / angulaires

# ==========================================
# 2. CONFIGURATION GAZEBO
# ==========================================
TOPIC = "/model/plateforme_hexapode/cmd_vel"
node = Node()
publisher = node.advertise(TOPIC, Twist)

if not publisher:
    print(f"Erreur : Impossible de publier sur {TOPIC}")
    exit()

print("Démarrage de la simulation hydrodynamique en temps réel...")
start_time = time.time()
t_precedent = 0.0

try:
    while True:
        t_actuel = time.time() - start_time
        dt = t_actuel - t_precedent
        
        # Sécurité pour éviter un dt nul ou aberrant au démarrage
        if dt <= 0:
            time.sleep(0.01)
            continue
            
        # --- CALCUL PHYSIQUE (Pas d'intégration) ---
        F = force_houle(t_actuel)
        
        # Somme des forces : F_net = F_houle - C*Vitesse - K*Position
        forces_restantes = F - np.dot(C, dSdt) - K * S
        
        # Accélération : linalg.solve est plus propre que l'inversion de matrice
        d2Sdt2 = np.linalg.solve(M_tot, forces_restantes)
        
        # Mise à jour de l'état (Intégration d'Euler)
        dSdt += d2Sdt2 * dt  # Nouvelle vitesse
        S += dSdt * dt       # Nouvelle position

        # --- ENVOI À GAZEBO ---
        msg = Twist()
        
        # Vitesse linéaire en Z (pilonnement) -> indice 2
        msg.linear.z = dSdt[2]
        
        # Vitesse angulaire en Y (tangage / pitch) -> indice 4
        msg.angular.y = dSdt[4]
        
        # Si tu veux ajouter le Roulis (X) -> indice 3
        msg.angular.x = dSdt[3]

        publisher.publish(msg)
        
        # Sauvegarde du temps pour le prochain calcul de dt
        t_precedent = t_actuel
        
        # Fréquence de la boucle (~50 Hz)
        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nArrêt et immobilisation de la plateforme...")
    stop_msg = Twist()
    publisher.publish(stop_msg)