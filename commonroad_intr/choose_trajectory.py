from planner_params import compute_trajectory_pair, convert_state_list_to_commonroad_object


def choose_trajectory(trajectories_all, planner_params, x_0, chosen_index):
    chosen_trajectory_pairs = []

    for trajectory in trajectories_all:
        trajectory_pair = compute_trajectory_pair(trajectory, planner_params, x_0) if trajectory is not None else None
        chosen_trajectory_pairs.append(trajectory_pair)

    chosen_trajectory_pair = chosen_trajectory_pairs[chosen_index]
    if chosen_trajectory_pair is not None:
        current_ego_vehicle = convert_state_list_to_commonroad_object(chosen_trajectory_pair[0].state_list,
                                                                      planner_params=planner_params)
    else:
        current_ego_vehicle = None
    return current_ego_vehicle, chosen_trajectory_pairs