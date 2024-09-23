import numpy as np
import math

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

    def update_body_state(self, msg):
        self.obs_dict[self.obs_terms[0]] = np.array(msg.vBody)
        self.obs_dict[self.obs_terms[1]] = np.array(msg.omegaBody)

        self.euler = np.array(msg.rpy)
        self.R = get_rotation_matrix_from_rpy(self.euler)
        self.obs_dict[self.obs_terms[2]] = np.dot(self.R.T, np.array([0, 0, -1])) # TODO
    
    def update_command(self, msg):
        # use velocity command: (x, y, yaw)
        self.obs_dict[self.obs_terms[3]] = np.array(msg.left_stick[1], msg.left_stick[0], msg.right_stick[0])

    def update_leg_state(self, msg):
        # update the leg state
        self.obs_dict[self.obs_terms[4]] = np.array(msg.q)
        self.obs_dict[self.obs_terms[5]] = np.array(msg.qd)

    def update_action(self, msg):
        # update the action
        self.obs_dict[self.obs_terms[6]] = np.array(msg.q_des)

    def update_camera_data(self, msg): # TODO
        pass

    def get_observation(self):
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
        