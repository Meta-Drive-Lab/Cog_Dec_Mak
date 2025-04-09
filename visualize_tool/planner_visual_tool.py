from copy import deepcopy

from tqdm import tqdm

from calculate_risk import get_main_prediction, calulate_risk
from choose_trajectory import choose_trajectory
from init_sample_params import init_sample_params, init_sample_params_in_trajectory
from init_scenario import ScenarioINTR
from planner_params import PlannerParams
from sample import sample_trajectories
from sampling_matrix import SamplingHandler
from state import ReactivePlannerState


class PlannerINTR:
    def __init__(self, file_path, timestep):
        self.chosen_trajectory_pairs = None
        self.obst_risk_list = None
        self.ego_risk_list = None
        self.ego_vehicle = None
        self.chosen_trajectory = None
        self.trajectories_all = None
        self.sampling_handler = None
        self.planner_params = None
        self.x_0: ReactivePlannerState = None
        self.planner_timestep = timestep
        self.scenario_intr = ScenarioINTR(file_path)
        self.scenario_intr.update_scenario(self.planner_timestep)

    def init_state(self, ref_index_func, input_velocity=None):
        # 直接在 ref_path 上画车和沿着规划的轨迹走的第一步都是用这个函数

        self.planner_params = PlannerParams(self.scenario_intr.coordinate_system,
                                       self.scenario_intr.planning_problem.initial_state.velocity)
        if ref_index_func >= len(self.scenario_intr.coordinate_system.ref_theta):
            raise ValueError("ref_index_func is out of scope for ref_theta")
        self.planner_params.theta_0 = self.scenario_intr.coordinate_system.ref_theta[ref_index_func]

        self.sampling_handler = SamplingHandler(dt=0.1, max_sampling_number=self.planner_params.max_sampling_number,
                                           t_min=0.5, horizon=3.0,
                                           delta_d_max=2.5,
                                           delta_d_min=-2.5,
                                           d_ego_pos=False)

        self.x_0 = ReactivePlannerState.create_from_initial_state(
                deepcopy(self.scenario_intr.planning_problem.initial_state),
                self.planner_params.wheelbase,
                self.planner_params.wb_rear_axle
            )

        self.sampling_handler, sample_level, x_0_lon, x_0_lat = init_sample_params(self.scenario_intr.coordinate_system,
                                                                              ref_index_func, self.x_0,
                                                                              self.scenario_intr.planning_problem,
                                                                              self.planner_params, self.sampling_handler, input_velocity=input_velocity)
        self.trajectories_all = sample_trajectories(self.sampling_handler, sample_level, x_0_lon, x_0_lat, self.planner_params)

    def plan(self, traj_index):
        self.ego_vehicle = None
        if not isinstance(traj_index, int) or traj_index < 0:
            raise ValueError("traj_index must be a non-negative integer")

        current_ego_vehicle, chosen_trajectory_pairs = choose_trajectory(self.trajectories_all, self.planner_params, self.x_0,
                                                                           chosen_index=traj_index)
        self.chosen_trajectory_pairs = chosen_trajectory_pairs
        if not self.chosen_trajectory_pairs:
            print(f"Error: No tracks available")
            return
        if traj_index >= len(self.chosen_trajectory_pairs):
            print(f"Error: Unable to find trajectory corresponding to index {traj_index}")
            return
        chosen_trajectory = self.chosen_trajectory_pairs[traj_index][0]

        self.chosen_trajectory = chosen_trajectory
        self.ego_vehicle = current_ego_vehicle

        # Debug 信息
        print(f"trajectory chosen: {traj_index}, trajectory Length: {len(self.chosen_trajectory.state_list)}")
        self.set_risks()

    def update_optimal_trajectory(self, traj_index):
        """更新最优轨迹"""
        chosen_trajectory = self.chosen_trajectory_pairs[traj_index][0]

        self.chosen_trajectory = chosen_trajectory


    def move_to_next_state_and_update(self):
        """沿着选定轨迹前进并更新场景"""
        if self.chosen_trajectory is None:
            print('Error: No track has been planned yet, and you cannot move forward')
            return

        # 确保轨迹长度足够
        if len(self.chosen_trajectory.state_list) <= 3:
            print("Error: The selected track is too short to move forward")
            return

        # 更新时间步
        self.planner_timestep += 3

        # 选取新起点
        self.x_0: ReactivePlannerState = deepcopy(self.chosen_trajectory.state_list[3])
        self.x_0.time_step = 0 # 必须置零, 不然显示不出来主车

        # 更新场景
        self.scenario_intr.update_scenario(self.planner_timestep)

        # 更新规划器参数
        self.planner_params.update_params(self.x_0.velocity, self.x_0.orientation)

        # 重新初始化采样参数
        self.sampling_handler, sample_level, x_0_lon, x_0_lat = init_sample_params_in_trajectory(
            self.scenario_intr.coordinate_system, self.x_0, self.planner_params, self.sampling_handler
        )

        # 生成新的采样轨迹
        self.trajectories_all = sample_trajectories(self.sampling_handler, sample_level, x_0_lon, x_0_lat,
                                                    self.planner_params)

        print(f"Update to Time Step: {self.planner_timestep}, Generated {len(self.trajectories_all)} new trajectories")

    def set_risks(self):
        """设置风险评估处理器，确保所有必要变量已正确初始化"""
        # 检查 `self.scenario_intr`
        if self.scenario_intr is None:
            print("Error: Scenario object `self.scenario_intr` has not been initialized")
            return

        # 检查 `self.ego_vehicle`
        if self.ego_vehicle is None:
            print("Error: No ego has been selected, unable to set risk assessment")
            return

        # 检查 `self.planner_timestep`
        if self.planner_timestep is None:
            print("Error: Planner timestep `self.planner_timestep` is empty")
            return

        # 检查 `self.planner_params`
        if self.planner_params is None:
            print("Error: Planning parameters `self.planner_params` have not been initialized")
            return

        # 检查 `self.trajectories_all`
        if not self.trajectories_all:
            print("Error: The trajectory data is empty and risk assessment cannot be performed")
            return

        ego_risk_list = []
        obst_risk_list = []

        global_predictions = get_main_prediction(self.scenario_intr.scenario, self.planner_timestep,
                                                 self.planner_params)
        for trajectory in tqdm(self.trajectories_all, desc="Calculating Risks", ncols=80):
            ego_risk, obst_risk_max = calulate_risk(self.scenario_intr.scenario, self.planner_timestep,
                                                    self.planner_params,
                                                    self.ego_vehicle,
                                                    trajectory, global_predictions)
            ego_risk_list.append(ego_risk)
            obst_risk_list.append(obst_risk_max)

        self.ego_risk_list = ego_risk_list
        self.obst_risk_list = obst_risk_list
        print(f"Time step: {self.planner_timestep}, {len(self.trajectories_all)} new trajectories’ risk data has been generated")