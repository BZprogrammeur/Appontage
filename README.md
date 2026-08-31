# Simulation d'Appontage Autonome de Drone 
Ce projet propose un environnement de simulation complet sous **PX4**, **Gazebo** et **ROS 2** pour l'entraînement et la validation (Sim2Sim / Sim2Real) de modèles d'intelligence artificielle (Deep Reinforcement Learning) dédiés à l'appontage autonome d'un drone quadricoptère sur une plateforme mobile (hexapode / houle marine) en présence de perturbations aérodynamiques (vent de Dryden / rafales).


---

## Architecture Globale

- **Simulateur Physique :** Gazebo Sim (Monde `ocean` avec houle et plateforme mobile)
- **Pilote Automatique :** PX4-Autopilot (SITL en mode Offboard)
- **Middleware & Passerelle :** Micro-XRCE-DDS Agent pour la communication uORB ⟷ ROS 2
- **Contrôle & IA :** Nœuds ROS 2 (Python / C++) exécutant des politiques DRL (Stable-Baselines3) en vitesses ou vitesses angulaires (*body rates*)

---

## Prérequis & Dépendances

- Ubuntu 22.04 LTS
- Docker avec support NVIDIA Container Toolkit (`--gpus all`)
- Serveur d'affichage X11 configuré pour le rendu graphique depuis Docker

---

## Guide d'Installation et de Démarrage

### 1. Préparation de l'hôte (Affichage X11)
Sur votre machine hôte, autorisez l'accès au serveur X11 :
```bash
xhost +local:docker
```

### 2. Lancer le conteneur Docker
```bash
docker run -it --gpus all -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/data/stage_lemaistre_2026/projet_appontage/Projet_appontage:/home ubuntu:22.04
```
Il faut faire attention au fichier que l'on bind, ici il faut remplacer /mnt/data/stage_lemaistre_2026/projet_appontage/Projet_appontage par le dossier du projet qui se trouve sur la machine réelle. Dans l'image docker il n'y a que ros2 sur ubuntu22.04

### 3. Lancer la simulation
Le fichier launch.sim permet de lancer la simulation
```bash
/PX4-ROS2-Gazebo-Drone-Simulation-Template/launch_sim.sh
```

### 4. Arrêter les processus
Si on a mal fermé gazebo
```bash
pkill -9 -f gz && pkill -9 -f ruby && pkill -9 -f px4
```

### 5. Lancer microXRCE Agent
Il fait le lien entre ROS2 et PX4, en lançant la commande suivante dans un terminal on leur permet de communiquer.
```bash
cd /home/Micro-XRCE-DDS-Agent/build
./MicroXRCEAgent udp4 -p 8888
```

### 6. Lancer les nœuds ROS 2
```bash
source /root/ros2_humble/install/setup.bash
cd /home/PX4-ROS2-Gazebo-Drone-Simulation-Template/ws_ros2
source install/setup.bash
export LD_LIBRARY_PATH=/home/PX4-ROS2-Gazebo-Drone-Simulation-Template/ws_ros2/install/wind_msgs/lib:$LD_LIBRARY_PATH
```

Lancement du contrôleur clavier (téléopération de test) :
```bash
ros2 run my_offboard_ctrl control_keyboard
```

Ou lancement de l'exécuteur d'actions pour le modèle d'IA (lancez dans 3 terminaux) :
```bash
ros2 run my_offboard_ctrl telemetry_pub
ros2 run my_offboard_ctrl drone_controller
ros2 run my_offboard_ctrl action_speed
```

### 7. Configuration du proxy (si nécessaire)
Il se peut que le proxy de l'école bloque la communication entre le noeud telemetry et le noeud controller car il publie sur le réseau local. Dans ce cas ces commandes peuvent être utiles.

```bash
export no_proxy="localhost,127.0.0.1,0.0.0.0,:"
export NO_PROXY="localhost,127.0.0.1,0.0.0.0,:"
unset http_proxy
unset https_proxy
unset HTTP_PROXY
unset HTTPS_PROXY
```

## Exemple de commande pour envoyer du vent dans la simulation
Ouvrir un terminal et ne pas oublier de sourcer ROS2.

```bash
ros2 topic pub --once /wind/cmd wind_msgs/msg/WindCmd "{
  force_mean: {x: 3.0, y: 0.0, z: 0.0},
  force_variance: 0.5,
  torque_mean: {x: 0.5, y: 0.0, z: 0.0},
  torque_variance: 0.1,
  gust_force_magnitude: 0.0,
  gust_torque_magnitude: 0.3,
  gust_frequency: 0.2
}"
```

Pour tout remettre à 0 :
```bash
ros2 topic pub --once /wind/cmd wind_msgs/msg/WindCmd "{
  force_mean: {x: 0.0, y: 0.0, z: 0.0},
  torque_mean: {x: 0.0, y: 0.0, z: 0.0}
}"
```
## Pour envoyer de la houle
lancer simplement le programme sim_wave_v2, choisissez les paramètres directement dans le programme.

## Si PX4 n'est pas installé correctement sur l'image :
Il faut alors cloner un git contenant PX4  et l'installer en utilisant ces commandes

```bash
cd ~
git clone --recursive https://github.com/SathanBERNARD/PX4-ROS2-Gazebo-Drone-Simulation-Template.git
cd ~/PX4-ROS2-Gazebo-Drone-Simulation-Template
./install_px4_gz_ros2_for_ubuntu.sh
```

Ensuite il faut copier le fichier du monde gazebo ocean.sdf présent sur ce git dans /PX4-Autopilot/Tools/simulation/gz/worlds
Il faut également copier les dossiers plateforme_hexapode, mono_cam, x650, x650_base et x650_camera dans /PX4-Autopilot/Tools/simulation/gz/models
Ils sont tous présnets dans objet.zip. En cas de manque de mesh ou de texture ne pas hésiter à la retirer du fichier sdf du monde.
