import math
from typing import List

import numpy as np

from planner_params import PlannerParams
from trajectories import TrajectorySample, CartesianSample, CurviLinearSample
from utils_coordinate_system import interpolate_angle

from utils_params import _EPS
def check_feasibility(trajectories: List[TrajectorySample], planner_params:PlannerParams):

    # initialize lists for output trajectories
    # infeasible trajectory list is only used for visualization when self._draw_traj_set is True
    infeasible_invalid_count_kinematics = np.zeros(11)
    trajectory_list = list()

    # loop over list of trajectories
    for trajectory in trajectories:

        trajectory.feasible = True
        trajectory.valid = True

        # create time array and precompute time interval information
        t = np.round(np.arange(0, trajectory.trajectory_long.delta_tau + trajectory.dt, trajectory.dt), 5)
        t2 = np.round(np.power(t, 2), 10)
        t3 = np.round(np.power(t, 3), 10)
        t4 = np.round(np.power(t, 4), 10)
        t5 = np.round(np.power(t, 5), 10)

        # length of the trajectory sample (i.e., number of time steps. can be smaller than planning horizon)
        traj_len = len(t)

        # initialize long. (s) and lat. (d) state vectors
        s = np.zeros(planner_params.N + 1)
        s_velocity = np.zeros(planner_params.N + 1)
        s_acceleration = np.zeros(planner_params.N + 1)
        d = np.zeros(planner_params.N + 1)
        d_velocity = np.zeros(planner_params.N + 1)
        d_acceleration = np.zeros(planner_params.N + 1)

        # compute longitudinal position, velocity, acceleration from trajectory sample
        s[:traj_len] = trajectory.trajectory_long.calc_position(t, t2, t3, t4, t5)  # lon pos
        s_velocity[:traj_len] = trajectory.trajectory_long.calc_velocity(t, t2, t3, t4)  # lon velocity
        s_acceleration[:traj_len] = trajectory.trajectory_long.calc_acceleration(t, t2, t3)  # lon acceleration

        # s-enlargement of t-sampled trajectories
        for ext in range(traj_len, planner_params.N + 1):
            s[ext] = s[ext - 1] + trajectory.dt * s_velocity[traj_len - 1]
        s_velocity[traj_len:] = s_velocity[traj_len - 1]
        s_acceleration[traj_len:] = 0.0

        # At low speeds, we have to sample the lateral motion over the travelled distance rather than time.
        if not planner_params.LOW_VEL_MODE:
            d[:traj_len] = trajectory.trajectory_lat.calc_position(t, t2, t3, t4, t5)  # lat pos
            d_velocity[:traj_len] = trajectory.trajectory_lat.calc_velocity(t, t2, t3, t4)  # lat velocity
            d_acceleration[:traj_len] = trajectory.trajectory_lat.calc_acceleration(t, t2, t3)  # lat acceleration
        else:
            # compute normalized travelled distance for low velocity mode of lateral planning
            s1 = s[:traj_len] - s[0]
            s2 = np.square(s1)
            s3 = s2 * s1
            s4 = np.square(s2)
            s5 = s4 * s1

            # compute lateral position, velocity, acceleration from trajectory sample
            d[:traj_len] = trajectory.trajectory_lat.calc_position(s1, s2, s3, s4, s5)  # lat pos
            # in LOW_VEL_MODE d_velocity is actually d' (see Diss. Moritz Werling  p.124)
            d_velocity[:traj_len] = trajectory.trajectory_lat.calc_velocity(s1, s2, s3, s4)  # lat velocity
            d_acceleration[:traj_len] = trajectory.trajectory_lat.calc_acceleration(s1, s2, s3)  # lat acceleration

        # d-enlargement of t-sampled trajectories
        d[traj_len:] = d[traj_len - 1]
        d_velocity[traj_len:] = 0.0
        d_acceleration[traj_len:] = 0.0

        # precision for near zero velocities from evaluation of polynomial coefficients
        # set small velocities to zero
        if np.any(s_velocity < - _EPS):
            trajectory.valid = False
            infeasible_invalid_count_kinematics[10] += 1

        s_velocity[np.abs(s_velocity) < _EPS] = 0.0

        # Initialize trajectory state vectors
        # (Global) Cartesian positions x, y
        x = np.zeros(planner_params.N + 1)
        y = np.zeros(planner_params.N + 1)
        # (Global) Cartesian velocity v and acceleration a
        v = np.zeros(planner_params.N + 1)
        a = np.zeros(planner_params.N + 1)
        # Orientation theta: Cartesian (gl) and Curvilinear (cl)
        theta_gl = np.zeros(planner_params.N + 1)
        theta_cl = np.zeros(planner_params.N + 1)
        # Curvature kappa : Cartesian (gl) and Curvilinear (cl)
        kappa_gl = np.zeros(planner_params.N + 1)
        kappa_cl = np.zeros(planner_params.N + 1)


        infeasible_count_kinematics_traj = np.zeros(11)
        for i in range(0, len(s)):
            # compute orientations
            # see Appendix A.1 of Moritz Werling's PhD Thesis for equations
            if not planner_params.LOW_VEL_MODE:
                if s_velocity[i] > 0.001:
                    dp = d_velocity[i] / s_velocity[i]
                else:
                    dp = 0.
                # see Eq. (A.8) from Moritz Werling's Diss
                ddot = d_acceleration[i] - dp * s_acceleration[i]

                if s_velocity[i] > 0.001:
                    dpp = ddot / (s_velocity[i] ** 2)
                else:
                    dpp = 0.
            else:
                dp = d_velocity[i]
                dpp = d_acceleration[i]

            # factor for interpolation
            s_idx = np.argmax(planner_params.coordinate_system.ref_pos > s[i]) - 1
            if s_idx + 1 >= len(planner_params.coordinate_system.ref_pos):
                trajectory.feasible = False
                infeasible_count_kinematics_traj[3] = 1
                break
            s_lambda = (s[i] - planner_params.coordinate_system.ref_pos[s_idx]) / (
                        planner_params.coordinate_system.ref_pos[s_idx + 1] - planner_params.coordinate_system.ref_pos[s_idx])

            # compute curvilinear (theta_cl) and global Cartesian (theta_gl) orientation
            if s_velocity[i] > 0.001:
                # LOW VELOCITY MODE: dp = d_velocity[i]
                # HIGH VELOCITY MODE: dp = d_velocity[i]/s_velocity[i]
                theta_cl[i] = np.arctan2(dp, 1.0)

                theta_gl[i] = theta_cl[i] + interpolate_angle(
                    s[i],
                    planner_params.coordinate_system.ref_pos[s_idx],
                    planner_params.coordinate_system.ref_pos[s_idx + 1],
                    planner_params.coordinate_system.ref_theta[s_idx],
                    planner_params.coordinate_system.ref_theta[s_idx + 1])
            else:
                if planner_params.LOW_VEL_MODE:
                    theta_cl[i] = np.arctan2(dp, 1.0)
                    theta_gl[i] = theta_cl[i] + interpolate_angle(
                        s[i],
                        planner_params.coordinate_system.ref_pos[s_idx],
                        planner_params.coordinate_system.ref_pos[s_idx + 1],
                        planner_params.coordinate_system.ref_theta[s_idx],
                        planner_params.coordinate_system.ref_theta[s_idx + 1])
                else:
                    # in stillstand (s_velocity~0) and High velocity mode: assume vehicle keeps global orientation
                    theta_gl[i] = planner_params.theta_0 if i == 0 else theta_gl[i - 1]

                    theta_cl[i] = theta_gl[i] - interpolate_angle(
                        s[i],
                        planner_params.coordinate_system.ref_pos[s_idx],
                        planner_params.coordinate_system.ref_pos[s_idx + 1],
                        planner_params.coordinate_system.ref_theta[s_idx],
                        planner_params.coordinate_system.ref_theta[s_idx + 1])

            # Interpolate curvature of reference path k_r at current position
            k_r = (planner_params.coordinate_system.ref_curv[s_idx + 1] - planner_params.coordinate_system.ref_curv[s_idx]) * s_lambda + \
                  planner_params.coordinate_system.ref_curv[s_idx]
            # Interpolate curvature rate of reference path k_r_d at current position
            k_r_d = (planner_params.coordinate_system.ref_curv_d[s_idx + 1] - planner_params.coordinate_system.ref_curv_d[
                s_idx]) * s_lambda + \
                    planner_params.coordinate_system.ref_curv_d[s_idx]

            # compute global curvature (see appendix A of Moritz Werling's PhD thesis)
            oneKrD = (1 - k_r * d[i])
            cosTheta = math.cos(theta_cl[i])
            tanTheta = np.tan(theta_cl[i])

            kappa_gl[i] = (dpp + (k_r * dp + k_r_d * d[i]) * tanTheta) * cosTheta * ((cosTheta / oneKrD) ** 2) + \
                          (cosTheta / oneKrD) * k_r

            kappa_cl[i] = kappa_gl[i] - k_r

            # compute (global) Cartesian velocity
            v[i] = s_velocity[i] * (oneKrD / (math.cos(theta_cl[i])))

            # compute (global) Cartesian acceleration
            a[i] = s_acceleration[i] * (oneKrD / cosTheta) + ((s_velocity[i] ** 2) / cosTheta) * (
                    oneKrD * tanTheta * (kappa_gl[i] * (oneKrD / cosTheta) - k_r) - (
                    k_r_d * d[i] + k_r * dp))

            # **************************
            # Velocity constraint
            # **************************
            if v[i] < -_EPS:
                trajectory.feasible = False
                infeasible_count_kinematics_traj[4] = 1

            # **************************
            # Curvature constraint
            # **************************
            kappa_max = np.tan(planner_params.delta_max) / planner_params.wheelbase
            if abs(kappa_gl[i]) > kappa_max:
                trajectory.feasible = False
                infeasible_count_kinematics_traj[5] = 1

            # **************************
            # Yaw rate constraint
            # **************************
            yaw_rate = (theta_gl[i] - theta_gl[i - 1]) / planner_params.dT if i > 0 else 0.
            theta_dot_max = kappa_max * v[i]
            if abs(round(yaw_rate, 5)) > theta_dot_max:
                trajectory.feasible = False
                infeasible_count_kinematics_traj[6] = 1

            # **************************
            # Curvature rate constraint
            # **************************
            kappa_dot = (kappa_gl[i] - kappa_gl[i - 1]) / planner_params.dT if i > 0 else 0.
            if abs(kappa_dot) > 0.4:
                trajectory.feasible = False
                infeasible_count_kinematics_traj[7] = 1

            # **************************
            # Acceleration rate constraint
            # **************************
            v_switch = planner_params.v_switch
            a_max = planner_params.a_max * v_switch / v[i] if v[i] > v_switch else planner_params.a_max
            a_min = -planner_params.a_max
            if not a_min <= a[i] <= a_max:
                trajectory.feasible = False
                infeasible_count_kinematics_traj[8] = 1

        # if selected polynomial trajectory is feasible, store it's Cartesian and Curvilinear trajectory
        if trajectory.feasible:
            for i in range(0, len(s)):
                # compute (global) Cartesian position
                pos: np.ndarray = planner_params.coordinate_system.convert_to_cartesian_coords(s[i], d[i])
                if pos is not None:
                    x[i] = pos[0]
                    y[i] = pos[1]
                else:
                    trajectory.valid = False
                    infeasible_count_kinematics_traj[9] = 1
                    break

            if trajectory.feasible:
                # store Cartesian trajectory
                trajectory.cartesian = CartesianSample(x, y, theta_gl, v, a, kappa_gl,
                                                       kappa_dot=np.append([0], np.diff(kappa_gl)),
                                                       current_time_step=traj_len)

                # store Curvilinear trajectory
                trajectory.curvilinear = CurviLinearSample(s, d, theta_cl,
                                                           ss=s_velocity, sss=s_acceleration,
                                                           dd=d_velocity, ddd=d_acceleration,
                                                           current_time_step=traj_len)

                trajectory.actual_traj_length = traj_len

            trajectory_list.append(trajectory)

        infeasible_invalid_count_kinematics += infeasible_count_kinematics_traj


    return trajectory_list