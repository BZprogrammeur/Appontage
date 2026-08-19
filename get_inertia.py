"""
Calculateur de matrice d'inertie approximative que j'ai réalisé pour le drône x650. On suppose que le drône est
constitué d'un corps central lourd contenant la batterie et l'électronique et qu'il y a 4 moteurs légers excentrés. 
On néglige le poids du chassis et des bras.
"""
M_corps = 1.326  
m_moteur = 0.171 

# Dimensions du corps central (en mètres)
x_corps = 0.160  # longueur
y_corps = 0.160  # largeur
z_corps = 0.140  # hauteur

# Position des moteurs (entraxe 650mm -> 0.2298m du centre en X et Y)
r_x = 0.230
r_y = 0.230

# 1. Inertie du corps central (boite homogène)
Ixx_c = (1/12) * M_corps * (y_corps**2 + z_corps**2)
Iyy_c = (1/12) * M_corps * (x_corps**2 + z_corps**2)
Izz_c = (1/12) * M_corps * (x_corps**2 + y_corps**2)

# 2. Inertie des 4 moteurs (masses ponctuelles excentrées)
Ixx_m = 4 * m_moteur * (r_y**2)
Iyy_m = 4 * m_moteur * (r_x**2)
Izz_m = 4 * m_moteur * (r_x**2 + r_y**2)

# Inertie Totale
Ixx = Ixx_c + Ixx_m
Iyy = Iyy_c + Iyy_m
Izz = Izz_c + Izz_m

print(f"ixx: {Ixx}")
print(f"iyy: {Iyy}")
print(f"izz: {Izz}")