import numpy as np
from stable_baselines3 import PPO

model = PPO.load("ppo_drone_platform_pid_mobile_v6.zip")

etat = np.array([0.5, 0.5, 0.5, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001])

action, _state = model.predict(etat, deterministic=True)
print(action)