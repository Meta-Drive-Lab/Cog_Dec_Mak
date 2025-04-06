import numpy as np

from check_feasibility import check_feasibility
from polynomial_trajectory import QuarticTrajectory, QuinticTrajectory
from trajectories import TrajectorySample, TrajectoryBundle


def sample_trajectories(sampling_handler, sample_level, x_0_lon, x_0_lat, planner_params):
    trajectories = list()
    for t in sampling_handler.t_sampling.to_range(sample_level):
        # Longitudinal sampling for all possible velocities
        for v in sampling_handler.v_sampling.to_range(sample_level):
            trajectory_long = QuarticTrajectory(tau_0=0, delta_tau=t, x_0=np.array(x_0_lon), x_d=np.array([v, 0]))

            # Sample lateral end states (add x_0_lat to sampled states)
            if trajectory_long.coeffs is not None:
                for d in sampling_handler.d_sampling.to_range(sample_level).union({x_0_lat[0]}):
                    end_state_lat = np.array([d, 0.0, 0.0])
                    # SWITCHING TO POSITION DOMAIN FOR LATERAL TRAJECTORY PLANNING
                    if planner_params.LOW_VEL_MODE:
                        s_lon_goal = trajectory_long.evaluate_state_at_tau(t)[0] - x_0_lon[0]
                        if s_lon_goal <= 0:
                            s_lon_goal = t
                        trajectory_lat = QuinticTrajectory(tau_0=0, delta_tau=s_lon_goal, x_0=np.array(x_0_lat),
                                                           x_d=end_state_lat)

                    # Switch to sampling over t for high velocities
                    else:
                        trajectory_lat = QuinticTrajectory(tau_0=0, delta_tau=t, x_0=np.array(x_0_lat),
                                                           x_d=end_state_lat)
                    if trajectory_lat.coeffs is not None:
                        trajectory_sample = TrajectorySample(3.0, 0.1, trajectory_long, trajectory_lat,
                                                             len(trajectories))
                        trajectories.append(trajectory_sample)

    trajectory_bundle = TrajectoryBundle(trajectories)
    trajectories_all = check_feasibility(trajectory_bundle.trajectories, planner_params)
    return trajectories_all