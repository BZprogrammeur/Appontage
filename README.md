Ce projet consiste en la réalisation d'une simulation pour une validation sim2real et sim2sim d'un modèle d'IA contrôlant un drône


Bonne commande ocean :

PX4_SYS_AUTOSTART=4010   PX4_SIM_MODEL=gz_x500_mono_cam   PX4_GZ_MODEL_POSE="2,2,1,0,0,0.9"   PX4_GZ_WORLD=ocean   PX4-Autopilot/build/px4_sitl_default/bin/px4

xhost +local:docker

pkill -9 -f gz && pkill -9 -f ruby && pkill -9 -f px4

microXRCE :

cd /home/Micro-XRCE-DDS-Agent/build
./MicroXRCEAgent udp4 -p 8888

Node ros2 :

cd home/PX4-ROS2-Gazebo-Drone-Simulation-Template/ws_ros2
source install/local_setup.bash
ros2 run  my_offboard_ctrl control_keyboard

ros
Doc : 

https://journals.sagepub.com/doi/abs/10.1177/0954406217739647 #bateau chinois relié à une plateforme
https://www.scs-ingenierie.com/pdf/cours/Houles.pdf #type de houles en français
https://www.shf-lhb.org/articles/lhb/pdf/1986/03/lhb1986032.pdf fct transfert du bateau avec ref en bio
https://theses.hal.science/tel-00011323/file/these.pdf

https://sci-hub.fr/10.1007/s10846-017-0757-5 #etat de l'art atterissage
https://www.researchgate.net/publication/336019875_Dynamic_Landing_of_an_Autonomous_Quadrotor_on_a_Moving_Platform_in_Turbulent_Wind_Conditions
https://www.preprints.org/frontend/manuscript/299e67e2e68a0ff6d6e241a437014fbc/download_pub

https://github.com/QuadCtrl/quad-ctrl #DRL pour contrôle
https://arc.aiaa.org/doi/10.2514/6.2021-1018

https://www.researchgate.net/figure/Comparison-between-DRL-and-PID-controller_fig11_367638938 #comparaison DRL/PID
https://www.mdpi.com/1996-1073/15/8/2834 #reacteur

https://resiliencemedia.co/launching-drones-at-sea-has-a-landing-problem-waiv-robotics-thinks-its-solved-it/ #utilisation de plateforme pour stabiliser le bateau

Commande docker :

lancement : docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/data/stage_lemaistre_2026/projet_appontage/Projet_appontage:/home ubuntu:22.04

docker run -it --gpus all -v $(pwd):/home/noe.lemaistre/Téléchargements/Drone-landing-main drone-rl-redhat \
  python ppo_drone_obs_basic_tensorboard.py

si pb de proxy :

export no_proxy="localhost,127.0.0.1,0.0.0.0,:"
export NO_PROXY="localhost,127.0.0.1,0.0.0.0,:"
unset http_proxy
unset https_proxy
unset HTTP_PROXY
unset HTTPS_PROXY

ROS2 :
ros2 interface show px4_msgs/msg/VehicleAttitudeSetpoint

# Vent latéral + effet de roulis (roll autour de X)
ros2 topic pub --once /wind/cmd wind_msgs/msg/WindCmd "{
  force_mean: {x: 3.0, y: 0.0, z: 0.0},
  force_variance: 0.5,
  torque_mean: {x: 0.5, y: 0.0, z: 0.0},
  torque_variance: 0.1,
  gust_force_magnitude: 0.0,
  gust_torque_magnitude: 0.3,
  gust_frequency: 0.2
}"

# Tout remettre à zéro
ros2 topic pub --once /wind/cmd wind_msgs/msg/WindCmd "{
  force_mean: {x: 0.0, y: 0.0, z: 0.0},
  torque_mean: {x: 0.0, y: 0.0, z: 0.0}
}"

Sourcer : source /root/ros2_humble/install/setup.bash
source /home/PX4-ROS2-Gazebo-Drone-Simulation-Template/ws_ros2/install/setup.bash
export LD_LIBRARY_PATH=/home/PX4-ROS2-Gazebo-Drone-Simulation-Template/ws_ros2/install/wind_msgs/lib:$LD_LIBRARY_PATH