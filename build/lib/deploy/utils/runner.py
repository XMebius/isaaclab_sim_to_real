from deploy.utils.lcm_bridge import LCMBridge
from deploy.utils.observer import Observer
from deploy.utils.command import NoneVelCommand
from rsl_rl.modules import ActorCritic, ActorCriticRecurrent, EmpiricalNormalization
from rsl_rl.algorithms import PPO
import numpy as np
import threading
import torch
import time
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

class Runner():
    def __init__(self, logdir, agent_cfg, env_cfg) -> None:
        print("start to construct Runner...")
        self.logdir = logdir
        # config
        self.agent_cfg = obj_to_dict(agent_cfg)
        self.env_cfg = obj_to_dict(env_cfg)
        # set num_envs to 1
        self.env_cfg["scene"]["num_envs"] = 1
        self.alg_cfg = self.agent_cfg["algorithm"]
        self.policy_cfg = self.agent_cfg["policy"]

        self.device = 'cpu'

        print("constrcuting NoneVelCommand...")
        self.ctrl_command = NoneVelCommand()
        print("constrcuting Observer...")
        self.observer = Observer(self.env_cfg)
        print("constrcuting LCMBridge...")
        self.lcm_bridge = LCMBridge(self.ctrl_command, self.observer)    

        # inference loop
        self.thread = None
        self._stop_event = threading.Event()  # for stopping the thread
        self._lock = threading.Lock()   # for synchronization
        self.current_step = 0 

    def load_policy(self, load_optimizer = True) -> None:
        model_path = os.path.join(self.logdir, f"model_{self.agent_cfg["max_iterations"] - 1}.pt")
        if not os.path.isfile(model_path):
            raise FileNotFoundError(f"Model file {model_path} not found.")
            
        obs = self.observer.get_observation()
        self.num_obs = obs.shape[1] # TODO: make the obs as a 2-dims tensor
        num_critic_obs = self.num_obs
        self.num_actions = 12
        num_steps_per_env = self.agent_cfg["num_steps_per_env"]

        try:
            actor_critic_class = eval(self.policy_cfg.pop("class_name"))
            actor_critic: ActorCritic | ActorCriticRecurrent = actor_critic_class(
                self.num_obs, num_critic_obs, self.num_actions, **self.policy_cfg
            ).to(self.device)

            alg_class = eval(self.alg_cfg.pop("class_name"))  # PPO
            alg: PPO = alg_class(actor_critic, device=self.device, **self.alg_cfg)
            
            empirical_normalization = self.agent_cfg["empirical_normalization"]
            print("empirical_normalization: {}".format(empirical_normalization))
            obs_normalizer = EmpiricalNormalization(shape=[self.num_obs], until=1.0e8).to(self.device) if empirical_normalization else torch.nn.Identity()
            print("obs_normalizer: {}".format(obs_normalizer))
            
            alg.init_storage(
                self.env_cfg["scene"]["num_envs"],
                num_steps_per_env,
                [self.num_obs],
                [num_critic_obs],
                [self.num_actions],
            )
            print("[INFO]: num_envs: {}, num_steps_per_env: {}, num_obs: {}, num_critic_obs: {}, num_actions: {}".format(
                self.env_cfg["scene"]["num_envs"], num_steps_per_env, self.num_obs, num_critic_obs, num_actions
            ))

            # load policy
            loaded_dict = torch.load(model_path, weights_only=True)
            alg.actor_critic.load_state_dict(loaded_dict["model_state_dict"])
            if empirical_normalization:
                obs_normalizer.load_state_dict(loaded_dict["obs_norm_state_dict"])
            
            if load_optimizer:
                alg.optimizer.load_state_dict(loaded_dict["optimizer_state_dict"])

            # switch to eval mode
            alg.actor_critic.eval()
            if empirical_normalization:
                obs_normalizer.eval()
            
            if self.device is not None:
                alg.actor_critic.to(self.device)
            # get inference policy    
            self.policy = alg.actor_critic.act_inference
            if self.agent_cfg["empirical_normalization"]:
                if self.device is not None:
                    obs_normalizer.to(self.device)
                self.policy = lambda x: alg.actor_critic.act_inference(obs_normalizer(x))
            
            print("Model loaded successfully.")
            print(self.policy)
        except Exception as e:
            print(f"Error loading model: {e}")
     
    def calibrate(self, debug = True) -> None:
        # move the robot to the initial position and test passive state for safety
        self.lcm_bridge.receive()
        init_joint_pos = self.observer.get_init_joint_pos()
        joint_pos = self.observer.get_joint_pos()
        target_joint_goal = np.zeros(12)

        print(f"About to calibrate, the robot will stand up [Press R2 to calibrate...]")
        while not self.ctrl_command.pre_check_calibration():
            time.sleep(0.1)
        self.ctrl_command.reset_R2_pressed()
        print("Calibrating...")
        # discrete the target joint position
        joint_pos_seq = []
        joint_pos_offet = joint_pos - init_joint_pos
        while np.max(np.abs(joint_pos_offet - target_joint_goal)) > 0.01:
            joint_pos_offet -= np.clip((joint_pos_offet - target_joint_goal), -0.05, 0.05)
            joint_pos_seq.append(joint_pos_offet)
        
        # perform actions
        for action in joint_pos_seq:
            scaled_action = action * self.env_cfg["actions"]["scale"]
            self.lcm_bridge.send(scaled_action)

        # test safety check
        if debug:
            print("Testing passive state...")
            time.sleep(5)
            # TODO: make the dog sit down
            pass

    def run(self) -> None:
        # perform calibration before running
        self.calibrate()
        # wait for calibration
        print("Make sure the robot has been calibrated. Press L2 to activate the controlling...")
        while not self.ctrl_command.post_check_calibration():
            time.sleep(0.1)
        self.ctrl_command.reset_L2_pressed()
        print("Starting the robot...")

        # load inference model
        self.load_policy()

        try:
            # start inference loop
            self.thread = threading.Thread(target=self._run_inference_loop)
            self.thread.start()
            self.thread.join()
        except KeyboardInterrupt:
            print("KeyboardInterrupt: stopping the robot...")
            self.stop()        
       
    def _run_inference_loop(self) -> None:
        while not self._stop_event.is_set() and self.current_step < self.cfg.max_steps:
            with self._lock:
                try:
                    # receive the message from lcm bridge
                    self.lcm_bridge.receive()
                    # get observation from with or without velocity command
                    obs = self.observer.get_observation()
                    # perform inference TODO: check the info of action
                    action = self.policy(obs)
                    # TODO: safety check, stop and reset the robot
                    
                    # send action to lcm bridge
                    self.lcm_bridge.send(action)
                    self.current_step += 1  # Increment step count
                except Exception as e:
                    # catch any exception and print the error
                    print(f"error orcurred: {e}") 
                    self.stop()
            time.sleep(0.01)

        if self.current_step >= self.max_steps:
            print(f"Max steps {self.max_steps} reached. Stopping...")
            self.stop()

    def stop(self): # after max_steps, stop will be called
        """Stop the running thread and reset the robot."""
        self._stop_event.set()
        self.thread.join()
        print("Stopping robot...")
        self.reset_robot()

    def reset_robot(self) -> None:
        print("Resetting the robot to initial state...")


