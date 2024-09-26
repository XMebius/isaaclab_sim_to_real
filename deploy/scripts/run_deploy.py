"""Launch Isaac Sim Simulator first."""
from omni.isaac.lab.app import AppLauncher

app_launcher = AppLauncher(headless = True)
simulation_app = app_launcher.app

"""Load the necessary modules."""
from omni.isaac.lab.utils.io import load_pickle
from deploy.utils.runner import Runner
import os
import glob

ROOT_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), "../../"))

if __name__ == "__main__":
    # from deploy.lcm_types.aliengo_body_data_lcmt import aliengo_body_data_lcmt
    exp_name = "unitree_aliengo_rough"

    dirs = glob.glob(f"{ROOT_DIR}/runs/{exp_name}/*")
    logdir = sorted(dirs)[0]

    # agent config
    agent_cfg = load_pickle(os.path.join(logdir, "params/agent.pkl"))
    # env config
    env_cfg = load_pickle(os.path.join(logdir, "params/env.pkl"))

    # create the runner
    print("Creating runner...")
    runner = Runner(logdir, agent_cfg, env_cfg)
    # runner.run()
    print("Runner created.")
    # runner.run()
    simulation_app.close()