"""Launch Isaac Sim Simulator first."""
from omni.isaac.lab.app import AppLauncher

app_launcher = AppLauncher(headless = True)
simulation_app = app_launcher.app

"""Load the necessary modules."""
from omni.isaac.lab.utils.io import load_pickle
# from deploy.utils.command import NoneVelCommand
from deploy.lcm_types.aliengo_body_data_lcmt import aliengo_body_data_lcmt
from rsl_rl.modules import ActorCritic, ActorCriticRecurrent, EmpiricalNormalization
from rsl_rl.algorithms import PPO
import torch
import glob
import os

def obj_to_dict(obj):
    """convert object to dictionary"""
    if isinstance(obj, dict):
        return {k: obj_to_dict(v) for k, v in obj.items()}
    elif hasattr(obj, "__dict__"):
        # Convert the object's attributes to a dictionary
        return {
            k: obj_to_dict(v)
            for k, v in obj.__dict__.items()
            if not k.startswith('__') and not callable(v)
        }
    elif isinstance(obj, (list, tuple, set)):
        return [obj_to_dict(v) for v in obj]
    # For other types of objects, return them 
    else:
        return obj


def main():
    print("Current working directory:", os.getcwd())

    exp_name = "unitree_aliengo_rough"

    dirs = glob.glob(f"../../runs/{exp_name}/*")
    logdir = sorted(dirs, reverse=True)[0]
    print("Log directory:", logdir)
    
    # agent config
    agent_cfg = load_pickle(os.path.join(logdir, "params/agent.pkl"))
    agent_cfg = obj_to_dict(agent_cfg)
    # alg_cfg = vars(agent_cfg["algorithm"])
    # policy_cfg = vars(agent_cfg["policy"])
    alg_cfg = agent_cfg["algorithm"]
    policy_cfg = agent_cfg["policy"]
    # env config
    env_cfg = load_pickle(os.path.join(logdir, "params/env.pkl"))
    env_cfg = obj_to_dict(env_cfg)


    max_iterations = 1500
    model_path = os.path.join(logdir, f"model_{max_iterations - 1}.pt")
    if not os.path.isfile(model_path):
        raise FileNotFoundError(f"Model file {model_path} not found.")

    device = 'cpu'
    num_obs = 235
    num_critic_obs = num_obs    # critic is not used in observation
    num_actions = 12
    num_steps_per_env = agent_cfg["num_steps_per_env"]
    try:
        actor_critic_class = eval(policy_cfg.pop("class_name"))
        print("actor_critic_class: {}".format(actor_critic_class))
        print("policy_cfg.type: {}".format(type(policy_cfg)))
        actor_critic: ActorCritic | ActorCriticRecurrent = actor_critic_class(
            num_obs, num_critic_obs, num_actions, **policy_cfg
        ).to(device)

        alg_class = eval(alg_cfg.pop("class_name"))  # PPO
        print("alg_class: {}".format(alg_class))
        alg: PPO = alg_class(actor_critic, device=device, **alg_cfg)
        
        empirical_normalization = agent_cfg["empirical_normalization"]
        print("empirical_normalization: {}".format(empirical_normalization))
        obs_normalizer = EmpiricalNormalization(shape=[num_obs], until=1.0e8).to(device) if empirical_normalization else torch.nn.Identity()
        print("obs_normalizer: {}".format(obs_normalizer))
        alg.init_storage(
            env_cfg["scene"]["num_envs"],
            num_steps_per_env,
            [num_obs],
            [num_critic_obs],
            [num_actions],
        )
        print("num_envs: {}, num_steps_per_env: {}, num_obs: {}, num_critic_obs: {}, num_actions: {}".format(
            env_cfg["scene"]["num_envs"], num_steps_per_env, num_obs, num_critic_obs, num_actions
        ))

        # load policy
        loaded_dict = torch.load(model_path, weights_only=True)
        alg.actor_critic.load_state_dict(loaded_dict["model_state_dict"])
        if empirical_normalization:
            obs_normalizer.load_state_dict(loaded_dict["obs_norm_state_dict"])
        load_optimizer = True
        if load_optimizer:
            alg.optimizer.load_state_dict(loaded_dict["optimizer_state_dict"])
        # switch to eval mode
        alg.actor_critic.eval()
        if empirical_normalization:
            obs_normalizer.eval()
        
        if device is not None:
            alg.actor_critic.to(device)
        # get inference policy    
        policy = alg.actor_critic.act_inference
        if agent_cfg["empirical_normalization"]:
            if device is not None:
                obs_normalizer.to(device)
            policy = lambda x: alg.actor_critic.act_inference(obs_normalizer(x))
        
        print("Model loaded successfully.")
        # print model info
        # print(policy)
    except Exception as e:
        print(f"Error loading model: {e}")
        


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()