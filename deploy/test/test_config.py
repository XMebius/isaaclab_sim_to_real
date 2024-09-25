"""Launch Isaac Sim Simulator first."""
from omni.isaac.lab.app import AppLauncher

app_launcher = AppLauncher(headless = True)
simulation_app = app_launcher.app

"""Load the necessary modules."""
from omni.isaac.lab.utils.io import load_pickle
import numpy as np
import re
import glob
import os

def main():
    print("Current working directory:", os.getcwd())

    exp_name = "unitree_aliengo_rough"

    dirs = glob.glob(f"../../runs/{exp_name}/*")
    logdir = sorted(dirs, reverse=True)[0]
    print("Log directory:", logdir)
    
    # agent config
    agent_cfg = load_pickle(os.path.join(logdir, "params/agent.pkl"))
    # # env config
    env_cfg = load_pickle(os.path.join(logdir, "params/env.pkl"))

    # read the config
    # print_attributes(env_cfg)
    init_pos = env_cfg.scene.robot.init_state.joint_pos
    joint_names = [
    'FL_hip_joint', 'FR_hip_joint', 'RL_hip_joint', 'RR_hip_joint',
    'FL_thigh_joint', 'FR_thigh_joint', 'RL_thigh_joint', 'RR_thigh_joint',
    'FL_calf_joint', 'FR_calf_joint', 'RL_calf_joint', 'RR_calf_joint']

    # 创建关节位置数组
    positions_array = np.array([
        next(value for pattern, value in init_pos.items() if re.match(pattern, name))
        for name in joint_names
    ])
    print("初始位置数组:", positions_array)

if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()