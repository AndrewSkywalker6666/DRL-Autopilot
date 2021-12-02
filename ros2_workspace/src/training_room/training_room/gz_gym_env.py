import math
from scipy.stats import special_ortho_group as so
import numpy as np
import rclpy
import time
import sys
import socket
import pickle
import gym

# iris quadrotor
class iris_quad(gym.Env):
    def __init(self):
        todo = 1

# omni_hex platform
class omni_hex(gym.Env):
    
    def __init__(self):

        # define absolute maximums
        self.delta_t = 100              # trajectory delta t = 100 ms
        self.max_linear_velocity = 5    # 5 m/s for each axis
        self.max_angular_velocity = 1   # 1 rad/s for each axis
        self.max_arm_rotation = 8       # 8 rads for each arm
        self.max_linear_accel = 10      # 10 m/s^2 for each axis
        self.max_angular_accel = 3      # 3 rad/s^2 for each axis
        self.max_position_error = self.delta_t * 0.001 * self.max_linear_velocity

        # define states
        self.state_lower_bound = np.array([
            # execution time (ms)
            0,
            # position error to target, px, py, pz (meter) relative to world frame
            -self.max_position_error,
            -self.max_position_error,   
            -self.max_position_error,
            # attitude error expressed in rotation matrix, relative to world frame
            -1,   # R11
            -1,   # R21
            -1,   # R31
            -1,   # R12
            -1,   # R22
            -1,   # R32
            -1,   # R13
            -1,   # R23
            -1,   # R33
            # vehicle linear velocity, vx, vy, vz (m/s), relative to world frame
            -self.max_linear_velocity,
            -self.max_linear_velocity,
            -self.max_linear_velocity,
            # vehicle angular velocity, wx, wy, wz (rad/s), relative to world frame
            -self.max_angular_velocity,
            -self.max_angular_velocity,
            -self.max_angular_velocity
            # arm rotation (rad)
            -self.max_arm_rotation,
            -self.max_arm_rotation,
            -self.max_arm_rotation,
            -self.max_arm_rotation,
            -self.max_arm_rotation,
            -self.max_arm_rotation,
        ])

        self.state_upper_bound = -self.state_lower_bound
        self.state_upper_bound[0] = self.delta_t * 2 # leave some space for reward calculation

        self.observation_space = gym.spaces.Box(np.float32(self.state_lower_bound), np.float32(self.state_upper_bound))
        self.observation_dimension = len(self.state_lower_bound)
        
        self.state_desired = np.array([
            # delta time
            100,
            # position error = 0
            0,
            0,
            0,
            # rotation error = 0, R = eye
            1,
            0,
            0,
            0,
            1,
            0,
            0,
            0,
            1
        ])
        
        # define action range and action space
        self.action_lower_bound = np.array([
            # normalized thrust m1 to m6
            0,
            0,
            0,
            0,
            0,
            0,
            # arm rotation angular velocity (rad/second)
            -1,
            -1,
            -1,
            -1,
            -1,
            -1
        ])
        self.action_upper_bound = np.array([
            # normalized thrust m1 to m6
            1,
            1,
            1,
            1,
            1,
            1,
            # arm rotation angular velocity (rad/second)
            1,
            1,
            1,
            1,
            1,
            1
        ])
        self.action_space = gym.spaces.Box(np.float32(self.action_lower_bound), np.float32(self.action_upper_bound, dtype=np.float32))
        
    
    def reset(self):

        self.success_steps = 0
        self.steps = 0

        R_init = so.rvs(3)

        init_pos_x = 0
        init_pos_y = 0
        init_pos_z = 2 + self.max_position_error

        init_linear_vel_x = np.random.uniform(-self.max_linear_velocity, self.max_linear_velocity)
        init_linear_vel_y = np.random.uniform(-self.max_linear_velocity, self.max_linear_velocity)
        init_linear_vel_z = np.random.uniform(-self.max_linear_velocity, self.max_linear_velocity)
        init_angular_vel_x = np.random.uniform(-self.max_angular_velocity, self.max_angular_velocity)
        init_angular_vel_y = np.random.uniform(-self.max_angular_velocity, self.max_angular_velocity)
        init_angular_vel_z = np.random.uniform(-self.max_angular_velocity, self.max_angular_velocity)

        delta_pos_x = self.random_pos_delta(init_linear_vel_x, self.max_linear_velocity, self.max_linear_accel, self.delta_t * 0.001)
        delta_pos_y = self.random_pos_delta(init_linear_vel_y, self.max_linear_velocity, self.max_linear_accel, self.delta_t * 0.001)
        delta_pos_z = self.random_pos_delta(init_linear_vel_z, self.max_linear_velocity, self.max_linear_accel, self.delta_t * 0.001)

        R_delta = self.random_rotation_delta(
            init_angular_vel_x, init_angular_vel_y, init_angular_vel_z, self.max_angular_velocity, self.max_angular_accel, self.delta_t * 0.001
        )
        
        self.state = np.array([
            # time (ms)
            0,
            # position error
            delta_pos_x,
            delta_pos_y,
            delta_pos_z,
            # attitude error
            R_delta[0, 0],
            R_delta[1, 0],
            R_delta[2, 0],
            R_delta[0, 1],
            R_delta[1, 1],
            R_delta[2, 1],
            R_delta[0, 2],
            R_delta[1, 2],
            R_delta[2, 2],
            # linear velocity
            init_linear_vel_x,
            init_linear_vel_y,
            init_linear_vel_z,
            # angular velocity
            init_angular_vel_x,
            init_angular_vel_y,
            init_angular_vel_z,
            # arm rotation
            np.random.uniform(-math.pi, math.pi),
            np.random.uniform(-math.pi, math.pi),
            np.random.uniform(-math.pi, math.pi),
            np.random.uniform(-math.pi, math.pi),
            np.random.uniform(-math.pi, math.pi),
            np.random.uniform(-math.pi, math.pi)
        ])
        self.arm_rotation = self.state[19:]
        self.action = np.random.uniform(self.action_lower_bound, self.action_upper_bound)

        print("self.state=")
        print(self.state)
        print("self.state_desired=")
        print(self.state_desired)
        print("self.action=")
        print(self.action)
        print("self.arm_rotation=")
        print(self.arm_rotation)

    def constrained_accelerated_motion(self, v_init, v_max, accel_max, delta_t):
        # all units are assumed in SI

        # s_max, assume accel > 0
        if v_init + delta_t * accel_max <= v_max:
            s_max = v_init * delta_t + 0.5 * accel_max * pow(delta_t, 2)
        else:
            t_accel = (v_max - v_init) / accel_max
            s_max = v_init * t_accel + 0.5 * accel_max * pow(t_accel, 2) + v_max * (delta_t - t_accel)

        # s_min, assume accel < 0
        if v_init - delta_t * accel_max <= -v_max:
            s_min = v_init * delta_t - 0.5 * accel_max * pow(delta_t, 2)
        else:
            t_accel = (v_init + v_max) / accel_max
            s_min = v_init * t_accel - 0.5 * accel_max * pow(t_accel, 2) - v_max * (delta_t - t_accel)

        return s_min, s_max


    def random_pos_delta(self, v_init, v_max, accel_max, delta_t):
        
        s_min, s_max = self.constrained_accelerated_motion(v_init, v_max, accel_max, delta_t)

        return np.random.uniform(s_min, s_max)
        
    
    def random_rotation_delta(self, wx_init, wy_init, wz_init, w_max, accel_max, delta_t):
        
        # theta_x
        theta_x_min, theta_x_max = self.constrained_accelerated_motion(wx_init, w_max, accel_max, delta_t)
        theta_x = np.random.uniform(theta_x_min, theta_x_max)
        R_x = np.array([
            [1, 0, 0],
            [0, math.cos(theta_x), -math.sin(theta_x)],
            [0, math.sin(theta_x), math.cos(theta_x)]
        ])

        # theta_y
        theta_y_min, theta_y_max = self.constrained_accelerated_motion(wy_init, w_max, accel_max, delta_t)
        theta_y = np.random.uniform(theta_y_min, theta_y_max)
        R_y = np.array([
            [math.cos(theta_y), 0, math.sin(theta_y)],
            [0, 1, 0],
            [-math.sin(theta_y), 0, math.cos(theta_y)]
        ])

        # theta_z
        theta_z_min, theta_z_max = self.constrained_accelerated_motion(wz_init, w_max, accel_max, delta_t)
        theta_z = np.random.uniform(theta_z_min, theta_z_max)
        R_z = np.array([
            [math.cos(theta_z), -math.sin(theta_z), 0],
            [math.sin(theta_z), math.cos(theta_z), 0],
            [0, 0, 1]
        ])

        return np.dot(R_z, np.dot(R_y, R_x))


        



        
        

        

if __name__ == '__main__':
    omni_hex().reset()
