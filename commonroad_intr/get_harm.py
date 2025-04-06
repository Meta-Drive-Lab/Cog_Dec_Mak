"""Harm estimation function calling models based on risk json."""
import json
import os

import numpy as np
from commonroad.scenario.obstacle import ObstacleType
from risk_assessment.helpers.properties import get_obstacle_mass
from risk_assessment.utils.logistic_regression_symmetrical import get_protected_inj_prob_log_reg_reduced_sym, \
    get_protected_inj_prob_log_reg_ignore_angle


# Dictionary for existence of protective crash structure.
obstacle_protection = {
    ObstacleType.CAR: True,
    ObstacleType.TRUCK: True,
    ObstacleType.BUS: True,
    ObstacleType.BICYCLE: False,
    ObstacleType.PEDESTRIAN: False,
    ObstacleType.PRIORITY_VEHICLE: True,
    ObstacleType.PARKED_VEHICLE: True,
    ObstacleType.TRAIN: True,
    ObstacleType.MOTORCYCLE: False,
    ObstacleType.TAXI: True,
    ObstacleType.ROAD_BOUNDARY: None,
    ObstacleType.PILLAR: None,
    ObstacleType.CONSTRUCTION_ZONE: None,
    ObstacleType.BUILDING: None,
    ObstacleType.MEDIAN_STRIP: None,
    ObstacleType.UNKNOWN: False,
}


def load_harm_parameter_json():
    """
    Load the harm_parameters.json with model parameters.

    Returns:
        Dict: model parameters from parameter.json
    """
    parameter_config_path = 'harm_parameters.json'
    with open(parameter_config_path, "r") as f:
        jsondata = json.load(f)
    return jsondata


def get_harm(scenario, traj, predictions, planner_params, coeffs, timer):

    obstacle_ids = list(predictions.keys())


    ego_harm_traj = {}
    obst_harm_traj = {}

    # get the ego vehicle size

    for obstacle_id in obstacle_ids:
        # choose which model should be used to calculate the harm
        ego_harm_fun, obstacle_harm_fun = get_model(obstacle_id, scenario)
        # only calculate the risk as long as both obstacles are in the scenario
        pred_path = predictions[obstacle_id]['pos_list']
        pred_length = min(len(traj.cartesian.x) - 1, len(pred_path))
        if pred_length == 0:
            continue

        # get the size, the velocity and the orientation of the predicted
        # vehicle
        pred_size = (
            predictions[obstacle_id]['shape']['length']
            * predictions[obstacle_id]['shape']['width']
        )
        pred_v = np.array(predictions[obstacle_id]['v_list'], dtype=np.float64)


        # replace get_obstacle_mass() by get_obstacle_mass()
        # get the predicted obstacle vehicle mass
        obstacle_mass = get_obstacle_mass(
            obstacle_type=scenario.obstacle_by_id(obstacle_id).obstacle_type, size=pred_size
        )


        # calc the risk for every time step
        with timer.time_with_cm(
                "simulation/sort trajectories/calculate costs/calculate risk/"
                + "calculate harm/calculate PDOF simple"
        ):
            # crash angle between ego vehicle and considered obstacle [rad]
            pdof_array = predictions[obstacle_id]["orientation_list"][:pred_length] - traj.cartesian.theta[:pred_length] + np.pi
            rel_angle_array = np.arctan2(predictions[obstacle_id]["pos_list"][:pred_length, 1] - traj.cartesian.y[:pred_length],
                                          predictions[obstacle_id]["pos_list"][:pred_length, 0] - traj.cartesian.x[:pred_length])
            # angle of impact area for the ego vehicle
            ego_angle_array = rel_angle_array - traj.cartesian.theta[:pred_length]
            # angle of impact area for the obstacle
            obs_angle_array = np.pi + rel_angle_array - predictions[obstacle_id]["orientation_list"][:pred_length]

            # calculate the difference between pre-crash and post-crash speed
            delta_v_array = np.sqrt(
                np.power(traj.cartesian.v[:pred_length], 2)
                + np.power(pred_v[:pred_length], 2)
                + 2 * traj.cartesian.v[:pred_length] * pred_v[:pred_length] * np.cos(pdof_array)
            )
            ego_delta_v = obstacle_mass / (planner_params.mass + obstacle_mass) * delta_v_array
            obstacle_delta_v = planner_params.mass / (planner_params.mass + obstacle_mass) * delta_v_array

            # calculate harm based on selected model
            ego_harm_obst = ego_harm_fun(velocity=ego_delta_v, angle=ego_angle_array, coeff=coeffs)
            obst_harm_obst = obstacle_harm_fun(velocity=obstacle_delta_v, angle=obs_angle_array, coeff=coeffs)
        # store harm list for the obstacles in dictionary for current frenét
        # trajectory
        ego_harm_traj[obstacle_id] = ego_harm_obst
        obst_harm_traj[obstacle_id] = obst_harm_obst

    return ego_harm_traj, obst_harm_traj


def get_model(obstacle_id, scenario):

    # obstacle protection type
    obs_protection = obstacle_protection[scenario.obstacle_by_id(obstacle_id).obstacle_type]

    # select case based on protection structure
    if obs_protection is True:
        # calculate harm
        ego_harm = get_protected_inj_prob_log_reg_reduced_sym
        # calculate harm for the obstacle vehicle
        obstacle_harm = get_protected_inj_prob_log_reg_reduced_sym

    elif obs_protection is False:
        # calc ego harm
        ego_harm = get_protected_inj_prob_log_reg_ignore_angle
        # calculate obstacle harm
        # logistic regression model
        obstacle_harm = lambda velocity,angle,coeff : 1 / (  # noqa E731
            1
            + np.exp(
                coeff["pedestrian"]["const"]
                - coeff["pedestrian"]["speed"] * velocity
            )
        )
    else:
        ego_harm = lambda velocity,angle,coeff : 1  # noqa E731
        obstacle_harm = lambda velocity,angle,coeff : 1  # noqa E731

    return ego_harm, obstacle_harm
