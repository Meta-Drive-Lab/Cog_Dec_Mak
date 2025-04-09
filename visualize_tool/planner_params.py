from typing import List

import numpy as np
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.state import InitialState, CustomState
from commonroad.scenario.trajectory import Trajectory
from commonroad.visualization.draw_params import DynamicObstacleParams, MPDrawParams

from state import ReactivePlannerState
from trajectories import TrajectorySample
from utils_coordinate_system import CoordinateSystem


class PlannerParams:
    def __init__(self, coordinate_system:CoordinateSystem, velocity):
        self.v_limit = 36
        self.planning_horizon = 3.0
        self.dt = 0.1
        self.dT = 0.1
        self.max_sampling_number = 3
        self.t_min = 1.1
        self.d_ego_pos = False
        self.N = int(self.planning_horizon / self.dt)
        self.low_vel_mode_threshold = 2.0
        if velocity < self.low_vel_mode_threshold:
            self.LOW_VEL_MODE = True
        else:
            self.LOW_VEL_MODE = False
        self.coordinate_system = coordinate_system
        self.delta_max = 1.066
        self.delta_min = -1.066
        self.wheelbase = 2.5789128
        self.v_switch = 7.319
        self.a_max = 11.5
        self.v_max = 50.8
        self.theta_0 = None
        self.length = 4.508
        self.width = 1.61
        self.wb_rear_axle = 1.4227170936
        self.mass = 1093.2952334674046

    def update_params(self, velocity, orientation):
        if velocity < self.low_vel_mode_threshold:
            self.LOW_VEL_MODE = True
        else:
            self.LOW_VEL_MODE = False
        self.theta_0 = orientation

def convert_state_list_to_commonroad_object(state_list: List[ReactivePlannerState], obstacle_id: int = 42, planner_params:PlannerParams = None):

    # shift trajectory positions to center
    new_state_list = list()
    for state in state_list:
        new_state_list.append(state.shift_positions_to_center(planner_params.wb_rear_axle))
    # new_state_list = state_list
    trajectory = Trajectory(initial_time_step=new_state_list[0].time_step, state_list=new_state_list)
    # get shape of vehicle
    shape = Rectangle(planner_params.length, planner_params.width)
    # get trajectory prediction
    prediction = TrajectoryPrediction(trajectory, shape)

    init_state = trajectory.state_list[0]
    initial_state = InitialState(position=init_state.position,
                                 orientation=init_state.orientation,
                                 velocity=init_state.velocity,
                                 acceleration=init_state.acceleration,
                                 yaw_rate=init_state.yaw_rate,
                                 slip_angle=0.0,
                                 time_step=init_state.time_step)

    return DynamicObstacle(obstacle_id, ObstacleType.CAR, shape, initial_state, prediction)

def compute_trajectory_pair(trajectory: TrajectorySample, planner_params:PlannerParams = None, x_0:ReactivePlannerState = None) -> tuple:

    # go along state list
    cart_list = list()
    cl_list = list()

    lon_list = list()
    lat_list = list()
    for i in range(len(trajectory.cartesian.x)):
        # create Cartesian state
        cart_states = dict()
        cart_states['time_step'] = x_0.time_step+i
        cart_states['position'] = np.array([trajectory.cartesian.x[i], trajectory.cartesian.y[i]])
        cart_states['orientation'] = trajectory.cartesian.theta[i]
        cart_states['velocity'] = trajectory.cartesian.v[i]
        cart_states['acceleration'] = trajectory.cartesian.a[i]
        if i > 0:
            cart_states['yaw_rate'] = (trajectory.cartesian.theta[i] - trajectory.cartesian.theta[i-1]) / planner_params.dT
        else:
            cart_states['yaw_rate'] = x_0.yaw_rate
        # TODO Check why computation with yaw rate was faulty ??
        cart_states['steering_angle'] = np.arctan2(planner_params.wheelbase *
                                                   trajectory.cartesian.kappa[i], 1.0)
        cart_list.append(ReactivePlannerState(**cart_states))

        # create curvilinear state
        # TODO: This is not correct
        cl_states = dict()
        cl_states['time_step'] = x_0.time_step+i
        cl_states['position'] = np.array([trajectory.curvilinear.s[i], trajectory.curvilinear.d[i]])
        cl_states['velocity'] = trajectory.cartesian.v[i]
        cl_states['acceleration'] = trajectory.cartesian.a[i]
        cl_states['orientation'] = trajectory.cartesian.theta[i]
        cl_states['yaw_rate'] = trajectory.cartesian.kappa[i]
        cl_list.append(CustomState(**cl_states))

        lon_list.append(
            [trajectory.curvilinear.s[i], trajectory.curvilinear.s_dot[i], trajectory.curvilinear.s_ddot[i]])
        lat_list.append(
            [trajectory.curvilinear.d[i], trajectory.curvilinear.d_dot[i], trajectory.curvilinear.d_ddot[i]])

    # make Cartesian and Curvilinear Trajectory
    cartTraj = Trajectory(x_0.time_step, cart_list)
    cvlnTraj = Trajectory(x_0.time_step, cl_list)

    # correct orientations of cartesian output trajectory
    cartTraj_corrected = shift_orientation(cartTraj, interval_start=x_0.orientation - np.pi,
                                                interval_end=x_0.orientation + np.pi)

    return cartTraj_corrected, cvlnTraj, lon_list, lat_list

def shift_orientation(trajectory: Trajectory, interval_start=-np.pi, interval_end=np.pi):
    for state in trajectory.state_list:
        while state.orientation < interval_start:
            state.orientation += 2 * np.pi
        while state.orientation > interval_end:
            state.orientation -= 2 * np.pi
    return trajectory


def define_visual_params(timestep):
    ego_params = DynamicObstacleParams()
    ego_params.time_begin = 0
    ego_params.draw_icon = True
    ego_params.show_label = False
    ego_params.vehicle_shape.occupancy.shape.facecolor = "#FFFFFF"
    ego_params.vehicle_shape.occupancy.shape.edgecolor = "#555555"
    ego_params.vehicle_shape.occupancy.shape.opacity = 1
    ego_params.vehicle_shape.occupancy.shape.zorder = 50
    ego_params.trajectory.draw_trajectory = False

    obs_params = MPDrawParams()
    obs_params.dynamic_obstacle.time_begin = timestep
    obs_params.dynamic_obstacle.draw_icon = False
    obs_params.dynamic_obstacle.show_label = False
    obs_params.dynamic_obstacle.vehicle_shape.occupancy.shape.facecolor = "#5685DF"
    obs_params.dynamic_obstacle.vehicle_shape.occupancy.shape.edgecolor = "#555555"

    obs_params.static_obstacle.show_label = False
    obs_params.static_obstacle.occupancy.shape.facecolor = "#a30000"
    obs_params.static_obstacle.occupancy.shape.edgecolor = "#756f61"

    return ego_params, obs_params