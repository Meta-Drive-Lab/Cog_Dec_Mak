from copy import deepcopy

import matplotlib
import numpy as np
from commonroad.visualization.mp_renderer import MPRenderer
from cr_scenario_handler.utils.prediction_helpers import main_prediction, prediction_preprocessing
from cr_scenario_handler.utils.sensor_model import get_obstacles_in_radius
from matplotlib import pyplot as plt
from prediction import WaleNet
from risk_assessment.helpers.timers import ExecTimer
from scipy.interpolate import griddata, splprep, splev

from calculate_risk import calulate_risk
from choose_trajectory import choose_trajectory
from get_harm import load_harm_parameter_json, get_harm
from get_ref_path import get_ref_path
from init_sample_params import init_sample_params
from init_scenario import init_scenario
from planner_params import PlannerParams, define_visual_params, compute_trajectory_pair
from sample import sample_trajectories
from sampling_matrix import SamplingHandler
from state import ReactivePlannerState
from utils_coordinate_system import CoordinateSystem


def plot_and_show(file_path_func, ref_index_func, timestep_func, ax_func, fig):
    scenario, planning_problem = init_scenario(file_path_func)
    reference_path = get_ref_path(scenario, planning_problem)
    coordinate_system = CoordinateSystem(reference=reference_path)

    planner_params = PlannerParams(coordinate_system, planning_problem.initial_state.velocity)
    planner_params.theta_0 = coordinate_system.ref_theta[ref_index_func]

    sampling_handler = SamplingHandler(dt=0.1, max_sampling_number=planner_params.max_sampling_number,
                                       t_min=0.5, horizon=3.0,
                                       delta_d_max=2.5,
                                       delta_d_min=-2.5,
                                       d_ego_pos=False)

    x_0 = ReactivePlannerState.create_from_initial_state(
                deepcopy(planning_problem.initial_state),
                planner_params.wheelbase,
                planner_params.wb_rear_axle
            )

    sampling_handler, sample_level, x_0_lon, x_0_lat = init_sample_params(coordinate_system, ref_index_func, x_0, planning_problem, planner_params, sampling_handler)
    trajectories_all = sample_trajectories(sampling_handler, sample_level, x_0_lon, x_0_lat, planner_params)

    # current_ego_vehicle1, chosen_trajectory_pairs1 = choose_trajectory(trajectories_all, planner_params, x_0, chosen_index=10)
    # chosen_trajectory_pair1 = chosen_trajectory_pairs1[10]

    current_ego_vehicle2, chosen_trajectory_pairs2 = choose_trajectory(trajectories_all, planner_params, x_0,
                                                                     chosen_index=23)
    chosen_trajectory_pair2 = chosen_trajectory_pairs2[23]

    ego_start_pos = current_ego_vehicle2.prediction.trajectory.state_list[0].position
    plot_limits = [-25 + ego_start_pos[0], 25 + ego_start_pos[0],
                               -25 + ego_start_pos[1], 25 + ego_start_pos[1]]
    rnd = MPRenderer(plot_limits=plot_limits, ax=ax_func, figsize=(7, 7))

    ego_params, obs_params = define_visual_params(timestep_func)
    scenario.draw(rnd, draw_params=obs_params)
    current_ego_vehicle2.draw(rnd, draw_params=ego_params)
    rnd.render()
    plt.plot(reference_path[:, 0], reference_path[:, 1], color='blue', linewidth=2.0, picker=False, zorder=18)

    # optimal_traj_positions = np.array(
    #     [(state.position[0], state.position[1]) for state in chosen_trajectory_pair1[0].state_list[0:]])
    # plt.plot(optimal_traj_positions[:, 0], optimal_traj_positions[:, 1], color='green', linewidth=2.0, picker=False,
    #          zorder=20, linestyle='solid')

    optimal_traj_positions = np.array(
        [(state.position[0], state.position[1]) for state in chosen_trajectory_pair2[0].state_list[0:]])
    # plt.plot(optimal_traj_positions[:, 0], optimal_traj_positions[:, 1], color='#00BFFF', linewidth=2.0, picker=False,
    #          zorder=21, linestyle='solid')

    start_point = optimal_traj_positions[0]
    point = plt.ginput(1)
    end_point = (point[0][0], point[0][1])

    x_points = np.linspace(start_point[0], end_point[0], num=10)
    y_points = np.linspace(start_point[1], end_point[1], num=10)

    tck, u = splprep([x_points, y_points], s=2)
    x_smooth, y_smooth = splev(np.linspace(0, 1, 100), tck)
    plt.plot(x_smooth, y_smooth, color='#00BFFF', linewidth=2, zorder=21)


    plt.axis('off')


def onclick(event):
    global end_point
    end_point = (event.xdata, event.ydata)
    # plt.close()

if __name__ == '__main__':
    file_path = "scenarios/ESP_Cambre-3_4_T-1.xml"
    timestep = 15
    ref_index = 60
    fig, ax = plt.subplots()
    fig.canvas.mpl_connect('button_press_event', onclick)
    plot_and_show(file_path, ref_index, timestep, ax, fig)

    # 保存高清图片
    plt.savefig("/Users/delvin/Desktop/0310/appendix2_3_big.png", dpi=300, bbox_inches='tight')
    plt.show()
    # 关闭图像，释放内存
    # plt.close(fig)
