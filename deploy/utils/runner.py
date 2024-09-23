from deploy.utils.lcm_bridge import LCMBridge
from deploy.utils.observer import Observer
from deploy.utils.command import NoneVelCommand

import pickle as pkl
import glob
import threading
import time

class Runner():
    def __init__(self, config) -> None:
        self.cfg = config

        self.ctrl_command = NoneVelCommand()
        self.observer = Observer(config)
        self.lcm_bridge = LCMBridge(self.ctrl_command, self.observer)
        
        self.policy = None

        self.logdir = None

        self.thread = None
        self._stop_event = threading.Event()  # for stopping the thread
        self._lock = threading.Lock()   # for synchronization
        self.current_step = 0 
        
    def load_policy(self) -> None:
        # get the path of logging
        # TODO: checkt the real path of the logdir
        dirs = glob.glob(f"../../runs/{self.cfg.exp_name}/*")
        self.logdir = sorted(dirs)[0]

        # load the actor model
        with open(self.logdir + "agent.pkl", "rb") as f:
            self.policy = pkl.load(f)
            # TODO: check the info of policy

    def calibrate(self) -> None:
        # move the robot to the initial position and test passive state for safety
        pass

    def run(self) -> None:
        # perform calibration before running
        print("Calibrating the robot...")
        self.calibrate()
        # wait for calibration
        print("Make sure the robot has been calibrated. Press L2 to activate the controlling...")
        while not self.ctrl_command.check_calibration():
            time.sleep(0.1)
        
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