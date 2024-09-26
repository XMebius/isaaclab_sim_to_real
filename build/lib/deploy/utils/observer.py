import numpy as np
import math
import re

def get_rotation_matrix_from_rpy(rpy):
    """
    Get rotation matrix from the given quaternion.
    Args:
        q (np.array[float[4]]): quaternion [w,x,y,z]
    Returns:
        np.array[float[3,3]]: rotation matrix.
    """
    r, p, y = rpy
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(r), -math.sin(r)],
                    [0, math.sin(r), math.cos(r)]
                    ])

    R_y = np.array([[math.cos(p), 0, math.sin(p)],
                    [0, 1, 0],
                    [-math.sin(p), 0, math.cos(p)]
                    ])

    R_z = np.array([[math.cos(y), -math.sin(y), 0],
                    [math.sin(y), math.cos(y), 0],
                    [0, 0, 1]
                    ])

    rot = np.dot(R_z, np.dot(R_y, R_x))
    return rot

class Observer():
    def __init__(self, config) -> None:
        self.cfg = config

        self.obs_dict = dict()
        self.obs_terms = ['base_lin_vel', 'base_ang_vel', 'projected_gravity', 
                     'velocity_commands', 'joint_pos', 'joint_vel', 'actions',
                     'height_scan']
        self.obs_scale = [1, 1, 1, 1, 1, 1, 1, 1]
        self.obs_clip = [None, None, None, None, None, None, None, (-1.0, 1.0)]
        
        self.euler = np.zeros(3)
        self.R = np.eye(3)

        # get default leg state from config
        self.init_joint_pos = self._set_init_joint_pos()
        print("init_joint_pos:", self.init_joint_pos)

    def _set_init_joint_pos(self):
        # get init_pos as a dict
        init_pos = self.cfg.scene.robot.init_state.joint_pos
        
        joint_names = [
            'FL_hip_joint', 'FR_hip_joint', 'RL_hip_joint', 'RR_hip_joint',
            'FL_thigh_joint', 'FR_thigh_joint', 'RL_thigh_joint', 'RR_thigh_joint',
            'FL_calf_joint', 'FR_calf_joint', 'RL_calf_joint', 'RR_calf_joint'
        ]

        # match the joint names with the init_pos
        init_pos_array = np.array([
            next(value for pattern, value in init_pos.items() if re.match(pattern, name))
            for name in joint_names
        ])

        # num_envs: 1
        init_pos_array = np.expand_dims(init_pos_array, axis=0)

        return init_pos_array    
    
    def get_init_joint_pos(self):
        return self.init_joint_pos
    
    def get_joint_pos(self):
        return self.obs_dict[self.obs_terms[4]]
    
    def update_body_state(self, msg):
        self.obs_dict[self.obs_terms[0]] = np.array(msg.vBody).reshape(1, -1)
        self.obs_dict[self.obs_terms[1]] = np.array(msg.omegaBody).reshape(1, -1)

        self.euler = np.array(msg.rpy).reshape(1, -1)
        self.R = get_rotation_matrix_from_rpy(self.euler).reshape(1, -1)
        self.obs_dict[self.obs_terms[2]] = np.dot(self.R.T, np.array([0, 0, -1])).reshape(1, -1) # TODO
    
    def update_command(self, msg):
        # use velocity command: (x, y, yaw)
        self.obs_dict[self.obs_terms[3]] = np.array(msg.left_stick[1], msg.left_stick[0], msg.right_stick[0]).reshape(1, -1)

    def update_leg_state(self, msg):
        # update the leg state
        self.obs_dict[self.obs_terms[4]] = np.array(msg.q).reshape(1, -1)
        self.obs_dict[self.obs_terms[5]] = np.array(msg.qd).reshape(1, -1)

    def update_action(self, msg):
        # update the action
        self.obs_dict[self.obs_terms[6]] = np.array(msg.q_des).reshape(1, -1)

    def update_camera_data(self, msg): # TODO
        pass

    def get_observation(self):
        # TODO: check info of the observation
        return self.compute()

    def compute(self):
        """ Compute the observation from the observation dictionary.
        1. perform the clip operation if necessary
        2. perform the scaling operation if necessary
        3. concatenate the observation terms
        """
        obs = np.zeros(0)
        for obs_key, obs_val in self.obs_dict.items():
            idx = self.obs_terms.index(obs_key)
            if self.obs_clip[idx] is not None:
                obs_val = np.clip(obs_val, self.obs_clip[idx][0], self.obs_clip[idx][1])
            if self.obs_scale[idx] is not None:
                obs_val = obs_val * self.obs_scale[idx]
            obs = np.concatenate((obs, obs_val))    # 1 dimension

        return obs
        