import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# --- Paramètres du système ---
# M_tot : masse généralisée (masse + masse ajoutée)
M_tot = np.eye(6) * 5000.0
M_tot[0, 0] = 500000.0  # Masse totale en X (cavalement)
M_tot[1, 1] = 500000.0  # Masse totale en Y (embardée)
M_tot[5, 5] = 200.0  # Inertie en lacet (Rz)

# C : matrice d'amortissement hydrodynamique
C = np.eye(6) * 500.0
C[4, 4] = 100.0  # Amortissement en tangage (y)
C[3, 3] = 1000.0  # Amortissement en roulis (x)
C[5, 5] = 10000.0  # Amortissement en lacet (Rz)
C[2, 2] = 8000.0  # Amortissement en pilonnement (z)

# K : matrice de raideur hydrostatique
# La raideur n'agit pas sur x, y et lacet (indices 0, 1 et 5) car // à la surface de l'eau => pas de modif du volume immergé => pas de rappel hydrostatique
K = np.zeros((6, 6))
K[2, 2] = 100.0  # Raideur en pilonnement (z)
K[3, 3] = 300.0  # Raideur en roulis
K[4, 4] = 100.0  # Raideur en tangage

def force_houle(t):
    """
    Définit le vecteur des efforts hydrodynamiques d'excitation F(t).
    Ici, on simule une houle régulière agissant uniquement sur le pilonnement (z, indice 2).
    """
    F = np.zeros(6)
    omega = 0.4  # Pulsation de la houle en rad/s
    F[2] = 5000.0 * np.cos(omega * t)  # Force appliquée sur le pilonnement (z)
    F[4] = 100.0 * np.sin(omega * t)  # Moment appliqué sur le tangage (y)
    F[3] = 200.0 * np.sin(omega * t)  # Moment appliqué sur le roulis (x)
    return F

def equations_mouvement(t, y):
    """
    Fonction d'état pour le solveur ODE.
    y est un vecteur de dimension 12 contenant [S (6 positions), dS/dt (6 vitesses)]
    """
    S = y[0:6]
    dSdt = y[6:12]

    # Calcul des forces d'excitation à l'instant t
    F = force_houle(t)

    # Résolution de l'accélération : d2S/dt2 = M_tot^-1 * (F - C*dS/dt - K*S)
    forces_restantes = F - np.dot(C, dSdt) - np.dot(K, S)
    d2Sdt2 = np.linalg.solve(M_tot, forces_restantes)

    # On retourne le dérivé du vecteur d'état : [vitesses, accélérations]
    return np.concatenate((dSdt, d2Sdt2))

# --- Conditions initiales et résolution ---
# Le navire part de sa position de repos (S=0) avec une vitesse nulle (dSdt=0)

def get_DOF(affichage = True, create_file = True):
    y0 = np.zeros(12)

    # Intervalle de temps pour la simulation
    tmax = 200
    t_span = (0, tmax)
    t_eval = np.linspace(t_span[0], t_span[1], 100*tmax)  # 100 points par seconde

    # Résolution numérique du système
    solution = solve_ivp(equations_mouvement, t_span, y0, t_eval=t_eval, method='RK45')

    # --- Affichage des résultats ---
    temps = solution.t
    cavalement_x = solution.y[0]  # Indice 0 correspond à la translation x (cavèlement)
    embardee_y = solution.y[1]  # Indice 1 correspond à la translation y (embardée)
    pilonnement_z = solution.y[2]  # Indice 2 correspond à la translation z (pilonnement)
    Roulis_Rx = solution.y[3]  # Indice 3 correspond au roulis (Rx)
    Tangage_Ry = solution.y[4]  # Indice 4 correspond au tangage (Ry)
    Lacet_Rz = solution.y[5]  # Indice 5 correspond au lacet (Rz)

    if affichage:
        fig, axes = plt.subplots(6, 1, figsize=(12, 12), sharex=True)

        # Titres et labels pour chaque sous-graphique
        titles = [
            "Cavalement (x)",
            "Embardée (y)",
            "Pilonnement (z)",
            "Roulis (Rx)",
            "Tangage (Ry)",
            "Lacet (Rz)"
        ]
        colors = ["blue", "orange", "green", "red", "purple", "brown"]

        # Tracer chaque mouvement
        for i, ax in enumerate(axes):
            ax.plot(temps, solution.y[i], label=titles[i], color=colors[i])
            ax.set_ylabel("Déplacement (m ou rad)")
            ax.set_title(titles[i])
            ax.grid(True)
            ax.legend()

        # Ajouter un label commun pour l'axe x
        axes[-1].set_xlabel("Temps (s)")

        # Ajustement de l'espacement entre les sous-graphiques
        plt.tight_layout()
        plt.show()

    if create_file:
        temps_arr = np.round(temps * 100) / 100.0
        with open("DOF_houle.hexa", "w") as f:
            for i in range(len(temps)):
                f.write(f"{temps_arr[i]:.3f}\t{cavalement_x[i]:.6f}\t{embardee_y[i]:.6f}\t{pilonnement_z[i]:.6f}\t{(180/np.pi)*Roulis_Rx[i]:.6f}\t{(180/np.pi)*Tangage_Ry[i]:.6f}\t{(180/np.pi)*Lacet_Rz[i]:.6f}\n")

    return temps, cavalement_x, embardee_y, pilonnement_z, Roulis_Rx, Tangage_Ry, Lacet_Rz

get_DOF()

