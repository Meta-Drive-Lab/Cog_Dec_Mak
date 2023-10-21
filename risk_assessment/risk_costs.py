"""Risk cost function and principles of ethics of risk."""
import numpy as np
import math
from Cog_Dec_Mak.risk_assessment.Cog_AIF import generate_B_matrices
from Cog_Dec_Mak.planner.Frenet.utils.validity_checks import create_collision_object
from commonroad_dc.collision.trajectory_queries import trajectory_queries
from commonroad_helper_functions.utils.cubicspline import CubicSpline2D

from Cog_Dec_Mak.risk_assessment.harm_estimation import get_harm
from Cog_Dec_Mak.risk_assessment.collision_probability import (
    get_collision_probability_fast,
    get_inv_mahalanobis_dist
)
from Cog_Dec_Mak.risk_assessment.helpers.timers import ExecTimer
from Cog_Dec_Mak.planner.utils.responsibility import assign_responsibility_by_action_space, calc_responsibility_reach_set
from Cog_Dec_Mak.risk_assessment.utils.logistic_regression_symmetrical import get_protected_inj_prob_log_reg_ignore_angle


def generate_reward_space(
    traj,
    state_space,
    csp: CubicSpline2D,
    reward_space,
    predictions: dict,
    scenario,
    ego_id: int,
    vehicle_params,
    params,
    exec_timer=None,
):
    
    timer = ExecTimer(timing_enabled=False) if exec_timer is None else exec_timer

    modes = params['modes']
    coeffs = params['harm']

    with timer.time_with_cm(
        "simulation/sort trajectories/calculate costs/calculate risk/"
        + "collision probability"
    ):

        if modes["fast_prob_mahalanobis"]:
            coll_prob_dict = get_inv_mahalanobis_dist(
                traj=traj,
                predictions=predictions,
                vehicle_params=vehicle_params,
            )

        else:
            coll_prob_dict = get_collision_probability_fast(
                traj=traj,
                predictions=predictions,
                vehicle_params=vehicle_params,
            )
    
    # get harm for ego and obst
    ego_harm_traj, obst_harm_traj = get_harm(
        scenario, traj, predictions, ego_id, vehicle_params, modes, coeffs, timer
    )
    
    # initialize
    obst_risk_traj = {}
    obst_risk_max = {}
    ego_harm_max = {}
    obst_harm_max = {}
    keys = []
    state_num = state_space.raw * state_space.col

    for key in ego_harm_traj.keys():
        
        # calculate the risk of obstacle's predicted trajectory
        obst_risk_traj[key] = [
            obst_harm_traj[key][t] * coll_prob_dict[key][t]
            for t in range(len(obst_harm_traj[key]))
        ]
    
        pred_traj = predictions[key]['pos_list']
        pred_length = min(len(traj.t) - 1, len(pred_traj))
        
        if pred_length == 0:
            continue
        
        keys.append(key)
        # there may be some predictions without any trajectory (when the obstacle disappears due to exceeding time)
            
        # get x- and y-position of the predicted trajectory
        x = pred_traj[:, 0][0:pred_length]
        y = pred_traj[:, 1][0:pred_length]

        # for points of predictions_traj within the state_space
        for index_pred in range(pred_length):

            p = (x[index_pred],y[index_pred])

            # calculate the s and the shortest distance
            s_in_curve, d_min = csp.get_min_arc_length(p)

            # check whether the points of predicted trajectory lies within the state space
            if check_in(state_space, s_in_curve, d_min) and coll_prob_dict[key][index_pred] > 0.1: 
            
                sx, sy = csp.calc_position(s_in_curve) 

                # t-vector
                t_vector = [csp.sx(s_in_curve,1), csp.sy(s_in_curve,1)] 

                # position of state space
                index_s, index_d = get_index(p, state_space, s_in_curve, d_min, sx, sy, t_vector)

                # add values to reward space
                reward_space[index_s, index_d] += obst_risk_traj[key][index_pred] * coll_prob_dict[key][index_pred] * ego_harm_traj[key][index_pred]
            
            else:
                continue

        obst_risk_max[key] = max(obst_risk_traj[key])
        ego_harm_max[key] = max(ego_harm_traj[key])
        obst_harm_max[key] = max(obst_harm_traj[key])

    reward_space = np.array(reward_space)

    return reward_space, obst_risk_max, ego_harm_max, obst_harm_max, keys

def calc_Cog_risk(
    traj,
    state_space,
    predictions: dict,
    belief_space,
    ego_state,
    vehicle_params,
    road_boundary,
    keys,
    params,
    exec_timer=None,
    
):

    '''
    # Cog_AIF: M * R = V
    B = generate_B_matrices(state_space.raw, state_space.col)
    avg_B = np.sum(B, axis=2) / len(B[:,:,0])
    I = np.identity(state_num)
    cog_matrix = np.linalg.inv(I - (2 * avg_B))
    cog_matrix = np.array(cog_matrix)
    np.save('cog_matrix.npy', cog_matrix)
    '''

    # initialize
    ego_risk_traj = {}
    ego_risk_max = {}
    coeffs = params['harm']

    # calculate cognitive risk for each traj
    for key in keys:

        pred_traj = predictions[key]['pos_list']
        pred_length = min(len(traj.t) - 1, len(pred_traj))
        
        if pred_length == 0:
            continue
        
        # calculate the cog_risk for each traj based on reward_space
        for ti in range(pred_length):

            # index_s_traj in state_space
            index_s_traj = math.floor(np.abs(traj.s[ti] - traj.s[0]) / (state_space.grid_size_log))
            # index_d_traj in state_space
            index_d_traj = math.floor((state_space.width/2 + traj.d[ti]) / state_space.grid_size_lat)
            # total cognitive risk of the traj (considering ego_harm and coll_prob)
            value = belief_space[index_s_traj, index_d_traj]
            ego_risk_traj.setdefault(key, []).append(value)
        
        # calculate maximum cognitive risk for each traj
        ego_risk_max[key] = np.sum(ego_risk_traj[key]) * 10 / len(traj.t)

    # calculate boundary harm
    col_obj = create_collision_object(traj, vehicle_params, ego_state)

    leaving_road_at = trajectory_queries.trajectories_collision_static_obstacles(
        trajectories=[col_obj],
        static_obstacles=road_boundary,
        method="grid",
        num_cells=32,
        auto_orientation=True,
        )

    if leaving_road_at[0] != -1:
        coll_time_step = leaving_road_at[0] - ego_state.time_step
        coll_vel = traj.v[coll_time_step]

        boundary_harm = get_protected_inj_prob_log_reg_ignore_angle(
        velocity=coll_vel, coeff=coeffs
        )

    else:
        boundary_harm = 0
    
    return ego_risk_max, boundary_harm


# check whether position p is within the state space
def check_in(state_space, s_in_curve, d_min):

    if d_min < state_space.width/2 and (s_in_curve - state_space.start_s) < state_space.length and s_in_curve >= state_space.start_s :
        return True
    else:
        return False

# get state index in the d-s frame
def get_index(p, state_space, s_in_curve, d_min, sx, sy, t_vector):
    # initialize
    x = p[0]
    y = p[1]
    
    # index_s
    index_s = math.floor((s_in_curve - state_space.start_s) / (state_space.grid_size_log))

    # index_d 
    if np.cross([x - sx, y - sy], t_vector) > 0: 
        # P lies on the left sife of the curve
        index_d = math.floor((state_space.width/2 + d_min) / state_space.grid_size_lat)
    else: 
        # P lies on the right sife of the curve
        index_d = math.floor((state_space.width/2 - d_min) /state_space.grid_size_lat)

    return index_s, index_d


def get_bayesian_costs(ego_risk_max, obst_risk_max, boundary_harm):
    """
    Bayesian Principle.

    Calculate the risk cost via the Bayesian Principle for the given
    trajectory.

    Args:
        ego_harm_traj (Dict): Dictionary with collision data for all
            obstacles and all time steps.
        obst_harm_traj (Dict): Dictionary with collision data for all
            obstacles and all time steps.

    Returns:
        Dict: Risk costs for the considered trajectory according to the
            Bayesian Principle
    """
    if len(ego_risk_max) == 0:
        return 0

    return (sum(ego_risk_max.values()) + sum(obst_risk_max.values()) + boundary_harm) / (
        len(ego_risk_max) * 2
    )


def get_equality_costs(ego_risk_max, obst_risk_max):
    """
    Equality Principle.

    Calculate the risk cost via the Equality Principle for the given
    trajectory.

    Args:
        ego_harm_traj (Dict): Dictionary with collision data for all
            obstacles and all time steps.
        obst_harm_traj (Dict): Dictionary with collision data for all
            obstacles and all time steps.
        timestep (Int): Currently considered time step.
        equality_mode (Str): Select between normalized ego risk
            ("normalized") and partial risks ("partial").

    Returns:
        float: Risk costs for the considered trajectory according to the
            Equality Principle
    """
    if len(ego_risk_max) == 0:
        return 0

    return sum(
        [abs(ego_risk_max[key] - obst_risk_max[key]) for key in ego_risk_max]
    ) / len(ego_risk_max)


def get_maximin_costs(ego_risk_max, obst_risk_max, ego_harm_max, obst_harm_max, boundary_harm, eps=10e-10, scale_factor=10):
    """
    Maximin Principle.

    Calculate the risk cost via the Maximin principle for the given
    trajectory.

    Args:
        ego_harm_traj (Dict): Dictionary with collision data for all
            obstacles and all time steps.
        obst_harm_traj (Dict): Dictionary with collision data for all
            obstacles and all time steps.
        timestep (Int): Currently considered time step.
        maximin_mode (Str): Select between normalized ego risk
            ("normalized") and partial risks ("partial").

    Returns:
        float: Risk costs for the considered trajectory according to the
            Maximum Principle
    """
    if len(ego_harm_max) == 0:
        return 0

    # Set maximin to 0 if probability (or risk) is 0
    maximin_ego = [a * int(b < eps) for a, b in zip(ego_harm_max.values(), ego_risk_max.values())]
    maximin_obst = [a * int(bool(b < eps)) for a, b in zip(obst_harm_max.values(), obst_risk_max.values())]

    return max(maximin_ego + maximin_obst + [boundary_harm])**scale_factor


def get_ego_costs(ego_risk_max, boundary_harm):
    """
    Calculate the utilitarian ego cost for the given trajectory.

    Args:
        ego_harm_traj (Dict): Dictionary with collision data for all
            obstacles and all time steps.
        timestep (Int): Currently considered time step.

    Returns:
        Dict: Utilitarian ego risk cost
    """
    if len(ego_risk_max) == 0:
        return 0

    return sum(ego_risk_max.values()) + boundary_harm


def get_responsibility_cost(scenario, traj, ego_state, obst_risk_max, predictions, reach_set, mode="reach_set"):
    """Get responsibility cost.

    Args:
        obst_risk_max (_type_): _description_
        predictions (_type_): _description_
        mode (str) : "reach set" for reachable set mode, else assignement by space of action

    Returns:
        _type_: _description_

    """
    bool_contain_cache = None
    if "reach_set" in mode and reach_set is not None:
        resp_cost, bool_contain_cache = calc_responsibility_reach_set(traj, ego_state, reach_set)

    else:
        # Assign responsibility to predictions
        predictions = assign_responsibility_by_action_space(
            scenario, ego_state, predictions
        )
        resp_cost = 0

        for key in predictions:
            resp_cost -= predictions[key]["responsibility"] * obst_risk_max[key]

    return resp_cost, bool_contain_cache
