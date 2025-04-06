import numpy as np

from state import ReactivePlannerState
from utils_coordinate_system import CoordinateSystem


def init_sample_params(coordinate_system, ref_index, x_0, planning_problem, planner_params, sampling_handler, input_velocity=None):
    s_0 = coordinate_system.ref_pos[ref_index]
    pos = coordinate_system.convert_to_cartesian_coords(s_0, 0)

    x_0.position = pos
    x_0.orientation = coordinate_system.ref_theta[ref_index]
    x_0.velocity = planning_problem.initial_state.velocity if input_velocity is None else input_velocity

    sample_level = 2  # sampling_mim 2 sampling_max 3
    x_0_lon = np.array([s_0, x_0.velocity, 0])
    x_0_lat = np.array([0, 0, 0])

    min_v = max(0.001, x_0.velocity - planner_params.a_max * planner_params.planning_horizon)
    max_v = min(min(x_0.velocity + (planner_params.a_max / 6.0) * planner_params.planning_horizon, planner_params.v_limit),
        planner_params.v_max)

    sampling_handler.set_v_sampling(min_v, max_v)

    return sampling_handler, sample_level, x_0_lon, x_0_lat

def init_sample_params_in_trajectory(coordinate_system: CoordinateSystem, x_0: ReactivePlannerState, planner_params, sampling_handler):
    s_0, d_0 = coordinate_system.convert_to_curvilinear_coords(x_0.position[0], x_0.position[1])
    sample_level = 2  # sampling_mim 2 sampling_max 3
    x_0_lon = np.array([s_0, x_0.velocity, x_0.acceleration])
    x_0_lat = np.array([d_0, 0, 0])

    min_v = max(0.001, x_0.velocity - planner_params.a_max * planner_params.planning_horizon)
    max_v = min(min(x_0.velocity + (planner_params.a_max / 6.0) * planner_params.planning_horizon, planner_params.v_limit),
        planner_params.v_max)

    sampling_handler.set_v_sampling(min_v, max_v)

    return sampling_handler, sample_level, x_0_lon, x_0_lat