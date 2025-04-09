# from copy import deepcopy
#
# import numpy as np
# from matplotlib import pyplot as plt
#
# from get_ref_path import get_ref_path
# from init_sample_params import init_sample_params
# from init_scenario import init_scenario
# from planner_params import PlannerParams, compute_trajectory_pair
# from sample import sample_trajectories
# from sampling_matrix import SamplingHandler
# from state import ReactivePlannerState
# from utils_coordinate_system import CoordinateSystem
#
#
# def plot_and_show(file_path_func, ref_index_func, ax_func):
#     scenario, planning_problem = init_scenario(file_path_func)
#
#     sampling_handler = SamplingHandler(dt=0.1, max_sampling_number=3,
#                                        t_min=1.1, horizon=3.0,
#                                        delta_d_max=3,
#                                        delta_d_min=-3,
#                                        d_ego_pos=False)
#
#     reference_path = get_ref_path(scenario, planning_problem)
#     coordinate_system = CoordinateSystem(reference=reference_path)
#
#     planner_params = PlannerParams(coordinate_system, planning_problem.initial_state.velocity)
#     planner_params.theta_0 = coordinate_system.ref_theta[ref_index_func]
#
#     x_0 = ReactivePlannerState.create_from_initial_state(
#         deepcopy(planning_problem.initial_state),
#         planner_params.wheelbase,
#         planner_params.wb_rear_axle
#     )
#
#     sampling_handler, sample_level, x_0_lon, x_0_lat = init_sample_params(coordinate_system, ref_index_func, x_0, planning_problem, planner_params, sampling_handler)
#     trajectories_all = sample_trajectories(sampling_handler, sample_level, x_0_lon, x_0_lat, planner_params)
#
#     traj_dic = {}
#     for i, trajectory in enumerate(trajectories_all):
#         trajectory_pair = compute_trajectory_pair(trajectory, planner_params, x_0) if trajectory is not None else None
#         traj_points = []
#         for state in trajectory_pair[0].state_list[0:]:
#             traj_points.append(state.position)
#         optimal_traj_positions = np.array(
#             [(state.position[0], state.position[1]) for state in trajectory_pair[0].state_list[0:]])
#         ax_func.plot(optimal_traj_positions[:, 0], optimal_traj_positions[:, 1], color='green', linewidth=1.0, zorder=25)
#         traj_dic[i] = np.array(traj_points)
#
# if __name__ == '__main__':
#     file_path = "scenarios/ARG_Carcarana-6_4_T-1.xml"
#     ref_index = 65
#     fig, ax = plt.subplots()
#     plot_and_show(file_path, ref_index, ax)
#     plt.show()
#     print('done')

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QLabel
from matplotlib import pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np
from copy import deepcopy
from get_ref_path import get_ref_path
from init_sample_params import init_sample_params
from init_scenario import init_scenario
from planner_params import PlannerParams, compute_trajectory_pair
from sample import sample_trajectories
from sampling_matrix import SamplingHandler
from state import ReactivePlannerState
from utils_coordinate_system import CoordinateSystem

class MainWindow(QMainWindow):
    def __init__(self, file_path, ref_index):
        super().__init__()
        self.file_path = file_path
        self.ref_index = ref_index
        self.traj_dic = {}
        self.initUI()

    def initUI(self):
        self.setWindowTitle("Trajectory Selector")
        # self.setGeometry(100, 100, 800, 600)

        # Create a central widget and set the layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # Create a matplotlib figure and canvas
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvas(self.fig)
        layout.addWidget(self.canvas)

        # Add a label to display the chosen trajectory
        self.label = QLabel("Chosen traj: None")
        layout.addWidget(self.label)

        # Plot the trajectories
        self.plot_trajectories()

        # Connect the mouse click event
        self.canvas.mpl_connect('button_press_event', self.on_click)
        self.setFixedSize(1000, 800)

    def plot_trajectories(self):
        scenario, planning_problem = init_scenario(self.file_path)

        reference_path = get_ref_path(scenario, planning_problem)
        coordinate_system = CoordinateSystem(reference=reference_path)

        planner_params = PlannerParams(coordinate_system, planning_problem.initial_state.velocity)
        planner_params.theta_0 = coordinate_system.ref_theta[self.ref_index]

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

        sampling_handler, sample_level, x_0_lon, x_0_lat = init_sample_params(coordinate_system, self.ref_index,
                                                                              x_0, planning_problem, planner_params,
                                                                              sampling_handler, None)
        trajectories_all = sample_trajectories(sampling_handler, sample_level, x_0_lon, x_0_lat, planner_params)

        for i, trajectory in enumerate(trajectories_all):
            trajectory_pair = compute_trajectory_pair(trajectory, planner_params, x_0) if trajectory is not None else None
            traj_points = []
            for state in trajectory_pair[0].state_list[0:]:
                traj_points.append(state.position)
            optimal_traj_positions = np.array(
                [(state.position[0], state.position[1]) for state in trajectory_pair[0].state_list[0:]])
            self.ax.plot(optimal_traj_positions[:, 0], optimal_traj_positions[:, 1], color='green', linewidth=1.0, zorder=25)
            self.traj_dic[i] = np.array(traj_points)

        self.canvas.draw()

    def on_click(self, event):
        if event.inaxes is not None:
            click_point = np.array([event.xdata, event.ydata])
            min_dist = float('inf')
            chosen_traj = None

            for traj_id, traj_points in self.traj_dic.items():
                distances = np.linalg.norm(traj_points - click_point, axis=1)
                min_traj_dist = np.min(distances)
                if min_traj_dist < min_dist:
                    min_dist = min_traj_dist
                    chosen_traj = traj_id

            if chosen_traj is not None:
                self.label.setText(f"Chosen traj: {chosen_traj}")
            else:
                self.label.setText("Chosen traj: None")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    file_path = "scenarios/ESP_Cambre-3_4_T-1.xml"
    ref_index = 60
    mainWin = MainWindow(file_path, ref_index)
    mainWin.show()
    sys.exit(app.exec_())