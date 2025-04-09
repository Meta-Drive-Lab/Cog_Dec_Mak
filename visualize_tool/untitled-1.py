import sys
from collections import namedtuple

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from copy import deepcopy
import numpy as np
from commonroad.visualization.mp_renderer import MPRenderer
from sympy.physics.units import velocity

from calculate_risk import calulate_risk
from choose_trajectory import choose_trajectory
from get_ref_path import get_ref_path
from init_sample_params import init_sample_params
from init_scenario import init_scenario
from planner_params import PlannerParams, define_visual_params
from sample import sample_trajectories
from sampling_matrix import SamplingHandler
from state import ReactivePlannerState
from utils_coordinate_system import CoordinateSystem

TrajectoryData = namedtuple("TrajectoryData",
                                ["current_ego_vehicle", "scenario", "chosen_trajectory_pairs", "reference_path"])

def calc_scenario(file_path_func, ref_index_func, timestep_func, choose_traj_index, input_v):
    scenario, planning_problem = init_scenario(file_path_func)

    sampling_handler = SamplingHandler(dt=0.1, max_sampling_number=3,
                                       t_min=0.5, horizon=3.0,
                                       delta_d_max=2.5,
                                       delta_d_min=-2.5,
                                       d_ego_pos=False)

    reference_path = get_ref_path(scenario, planning_problem)
    coordinate_system = CoordinateSystem(reference=reference_path)

    planner_params = PlannerParams(coordinate_system, planning_problem.initial_state.velocity)
    planner_params.theta_0 = coordinate_system.ref_theta[ref_index_func]

    x_0 = ReactivePlannerState.create_from_initial_state(
        deepcopy(planning_problem.initial_state),
        planner_params.wheelbase,
        planner_params.wb_rear_axle
    )

    sampling_handler, sample_level, x_0_lon, x_0_lat = init_sample_params(coordinate_system, ref_index_func, x_0, planning_problem, planner_params, sampling_handler, input_v)
    trajectories_all = sample_trajectories(sampling_handler, sample_level, x_0_lon, x_0_lat, planner_params)

    current_ego_vehicle, chosen_trajectory_pairs = choose_trajectory(trajectories_all, planner_params, x_0, chosen_index=choose_traj_index)

    ego_risk, obst_risk_max = calulate_risk(scenario, timestep_func, planner_params, current_ego_vehicle,
                                            trajectories_all[choose_traj_index])

    scenario_data = TrajectoryData(
        current_ego_vehicle=current_ego_vehicle,
        scenario=scenario,
        chosen_trajectory_pairs=chosen_trajectory_pairs,
        reference_path=reference_path
    )

    return ego_risk, obst_risk_max, scenario_data


def plot_and_show(scenario_data, timestep_func, choose_traj_index, ax_func):
    ego_start_pos = scenario_data.current_ego_vehicle.prediction.trajectory.state_list[0].position
    plot_limits = [-30 + ego_start_pos[0], 30 + ego_start_pos[0],
                   -30 + ego_start_pos[1], 30 + ego_start_pos[1]]
    rnd = MPRenderer(plot_limits=plot_limits, ax=ax_func, figsize=(7, 7))

    ego_params, obs_params = define_visual_params(timestep_func)
    scenario_data.scenario.draw(rnd, draw_params=obs_params)
    scenario_data.current_ego_vehicle.draw(rnd, draw_params=ego_params)
    rnd.render()
    chosen_trajectory_pair = scenario_data.chosen_trajectory_pairs[choose_traj_index]
    # Plot the optimal trajectory
    if chosen_trajectory_pair and len(chosen_trajectory_pair[0].state_list) > 0:
        optimal_traj_positions = np.array(
            [(state.position[0], state.position[1]) for state in chosen_trajectory_pair[0].state_list[0:]])
        ax_func.plot(optimal_traj_positions[:, 0], optimal_traj_positions[:, 1], color='green', linewidth=2.0,
                     zorder=25)

    # Plot the reference path
    ax_func.plot(scenario_data.reference_path[:, 0], scenario_data.reference_path[:, 1], color='blue', linewidth=2.0, zorder=18)

class MainWindow(QWidget):
    def __init__(self, file_path):
        super().__init__()

        self.obst_risk = None
        self.ego_risk = None
        self.scenario_data = None
        self.obst_risk_list = None
        self.ego_risk_list = None
        self.file_path = file_path
        self.timestep = 5
        self.ref_index = 10
        self.traj_index = 10
        self.velocity = None
        self.initUI()

    def initUI(self):
        # Main layout
        main_layout = QHBoxLayout()

        # Left side: matplotlib figure
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        main_layout.addWidget(self.canvas)

        # Right side: controls
        controls_layout = QVBoxLayout()
        controls_layout.setAlignment(Qt.AlignTop)  # 控件靠上排列

        # Timestep input
        timestep_layout = QHBoxLayout()
        timestep_layout.addWidget(QLabel("Timestep:"))
        self.timestep_input = QLineEdit(str(self.timestep))
        timestep_layout.addWidget(self.timestep_input)
        controls_layout.addLayout(timestep_layout)

        # Ref index input
        ref_index_layout = QHBoxLayout()
        ref_index_layout.addWidget(QLabel("Ref Index:"))
        self.ref_index_input = QLineEdit(str(self.ref_index))
        ref_index_layout.addWidget(self.ref_index_input)
        controls_layout.addLayout(ref_index_layout)

        # Traj choose index input
        traj_index_layout = QHBoxLayout()
        traj_index_layout.addWidget(QLabel("Traj choose Index:"))
        self.traj_index_input = QLineEdit(str(self.traj_index))
        traj_index_layout.addWidget(self.traj_index_input)
        controls_layout.addLayout(traj_index_layout)

        # Velocity input
        velocity_layout = QHBoxLayout()
        velocity_layout.addWidget(QLabel("Velocity Input:"))
        self.velocity_input = QLineEdit(str(self.velocity))
        velocity_layout.addWidget(self.velocity_input)
        controls_layout.addLayout(velocity_layout)

        # Generate button
        self.generate_button = QPushButton("Generate")
        self.generate_button.clicked.connect(self.generate_plot)
        controls_layout.addWidget(self.generate_button)

        # Add labels to display ego_risk and obst_risk_max
        self.ego_risk_label = QLabel("ego_risk: ")
        controls_layout.addWidget(self.ego_risk_label)

        self.obst_risk_label = QLabel("obst_risk: ")
        controls_layout.addWidget(self.obst_risk_label)

        main_layout.addLayout(controls_layout)

        self.setLayout(main_layout)
        self.setWindowTitle('Trajectory Visualizer')
        self.setFixedSize(800, 600)
        self.show()

    def generate_plot(self):
        # Get the updated values from the input fields
        self.timestep = int(self.timestep_input.text())
        self.ref_index = int(self.ref_index_input.text())
        self.traj_index = int(self.traj_index_input.text())
        try:
            self.velocity = float(self.velocity_input.text())

        except ValueError:
            self.velocity = None

        # Generate the new plot
        self.ego_risk, self.obst_risk, self.scenario_data = calc_scenario(self.file_path, self.ref_index, self.timestep, self.traj_index, self.velocity)
        self.traj_index = int(self.traj_index_input.text())

        # Clear the previous plot
        self.figure.clear()

        ax = self.figure.add_subplot(111)
        plot_and_show(self.scenario_data, self.timestep, self.traj_index, ax)
        self.canvas.draw()

        if self.ego_risk is not None and self.obst_risk is not None:
            # Update the labels with the results
            self.ego_risk_label.setText(f"ego_risk: {self.ego_risk:.4f}")  # 显示 ego_risk，保留 4 位小数

            # Format obst_risk_max as a string
            obst_risk_str = "obst_risk:\n"
            for obstacle_id, risk_value in self.obst_risk.items():
                obst_risk_str += f"{obstacle_id}: {risk_value:.4f}\n"  # 每个障碍物的风险值，保留 4 位小数
            self.obst_risk_label.setText(obst_risk_str)


if __name__ == '__main__':
    import matplotlib
    matplotlib.use('MacOSX')  # 设置后端为 MacOSX
    app = QApplication(sys.argv)
    main_window = MainWindow("scenarios/DEU_Flensburg-96_1_T-1.xml")
    sys.exit(app.exec_())