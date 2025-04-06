import numpy as np
from cr_scenario_handler.utils.prediction_helpers import main_prediction
from cr_scenario_handler.utils.sensor_model import get_obstacles_in_radius
from prediction import WaleNet
from risk_assessment.helpers.timers import ExecTimer

from collision_probability import get_collision_probability_fast
from get_harm import load_harm_parameter_json, get_harm


def calulate_risk(scenario, timestep_func, planner_params, current_ego_vehicle, traj, global_predictions):

    visible_obstacles = get_obstacles_in_radius(
        scenario=scenario,
        ego_id=42,
        ego_state=current_ego_vehicle,
        time_step=timestep_func,
        radius=20,
        vehicles_in_cone_angle=True)

    predictions = {key: item for key, item in global_predictions.items() if key in visible_obstacles}
    collision_prob_dict = get_collision_probability_fast(traj, predictions, planner_params)

    params_harm = load_harm_parameter_json()
    timer = ExecTimer(timing_enabled=False)
    ego_harm_traj, obst_harm_traj = get_harm(
        scenario, traj, predictions, planner_params, params_harm, timer
    )

    # Calculate risk out of harm and collision probability
    ego_risk_traj = {}
    obst_risk_traj = {}
    ego_risk_max = {}
    obst_risk_max = {}
    ego_harm_max = {}
    obst_harm_max = {}

    # Pedestrian max harm at position of possible collision
    obst_harm_occ = {}

    for key in ego_harm_traj:
        ego_risk_traj[key] = [
            ego_harm_traj[key][t] * collision_prob_dict[key][t]
            for t in range(len(ego_harm_traj[key]))
        ]
        obst_risk_traj[key] = [
            obst_harm_traj[key][t] * collision_prob_dict[key][t]
            for t in range(len(obst_harm_traj[key]))
        ]
        if np.max(collision_prob_dict[key]) > 0.001:
            obst_harm_occ[key] = obst_harm_traj[key][np.argmax(collision_prob_dict[key])]
        else:
            obst_harm_occ[key] = 0.0
        # Take max as representative for the whole trajectory
        ego_risk_max[key] = sum(ego_risk_traj[key])
        obst_risk_max[key] = sum(obst_risk_traj[key])
        ego_harm_max[key] = sum(ego_harm_traj[key])
        obst_harm_max[key] = sum(obst_harm_traj[key])
    try:
        ego_risk = max(list(ego_risk_max.values()))
    except:
        ego_risk = 0
    return ego_risk, obst_risk_max

def get_main_prediction(scenario, timestep_func, planner_params):
    predictor = WaleNet(scenario=scenario, mpl_backend="MacOSX")
    obstacle_list = [obs.obstacle_id for obs in scenario.obstacles]

    global_predictions = main_prediction(predictor, scenario, obstacle_list,
                                         timestep_func, scenario.dt,
                                         [planner_params.planning_horizon])
    return global_predictions