import sys

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QIntValidator, QDoubleValidator
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QGroupBox, \
    QFormLayout, QTextEdit

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from msg_logger import StdoutRedirector
from planner_visual_tool import PlannerINTR
from visualization import ScenarioVisualization


def find_min_risk_trajectory_for_vehicle(veh_id, ego_risk_list, obst_risk_list):
    if veh_id == 42:
        sorted_indices = sorted(range(len(ego_risk_list)), key=lambda i: ego_risk_list[i])

        min_indices = sorted_indices[:5]
        return min_indices
    else:
        veh_risk_list = [risk[veh_id] for risk in obst_risk_list]
        sorted_indices = sorted(range(len(veh_risk_list)), key=lambda i: veh_risk_list[i])

        min_indices = sorted_indices[:5]

    return min_indices

def find_max_risk_trajectory_for_vehicle(veh_id, ego_risk_list, obst_risk_list):
    if veh_id == 42:
        max_value = max(ego_risk_list)
        max_indices = [i for i, v in enumerate(ego_risk_list) if v == max_value]
        return max_indices
    else:
        veh_risk_list = [risk[veh_id] for risk in obst_risk_list]
        max_value = max(veh_risk_list)
        max_indices = [i for i, v in enumerate(veh_risk_list) if v == max_value]
        return max_indices

def find_min_sum_risk_indices(ego_risk_list, obst_risk_list):
    total_risk_list = [ego_risk + sum(obst_risk.values()) for ego_risk, obst_risk in zip(ego_risk_list, obst_risk_list)]

    sorted_indices = sorted(range(len(total_risk_list)), key=lambda i: total_risk_list[i])

    min_indices = sorted_indices[:5]

    return min_indices


class MainWindow(QWidget):
    def __init__(self, file_path):
        super().__init__()

        self.canvas = None
        self.figure = None
        self.planner = None
        self.scenario_data = None
        self.obst_risk_list = None
        self.ego_risk_list = None
        self.file_path = file_path
        self.timestep = 15
        self.ref_index = 60
        self.traj_index = 10
        self.velocity = None
        self.veh_id = 42

        self.timestep_input = QLineEdit(str(self.timestep))
        self.ref_index_input = QLineEdit(str(self.ref_index))
        self.velocity_input = QLineEdit(str(self.velocity))
        self.veh_input = QLineEdit(str(self.veh_id))

        self.generate_button = QPushButton("Calculate Scenario")
        self.plot_button = QPushButton("Plot")
        self.show_risk_button = QPushButton("Risk Show")
        self.move_button = QPushButton("Move Forward")

        self.ego_risk_label = QLabel("ego_risk: ")
        self.obst_risk_label = QLabel("obst_risk: ")

        self.output_text_edit = QTextEdit(self)
        self.output_text_edit.setReadOnly(True)
        self.timestep_input.setValidator(QIntValidator(0, 1000))

        self.velocity_input.setValidator(QDoubleValidator(0.0, 100.0, 2))

        self.init_ui()

        self.generate_button.clicked.connect(self.generate_plot)
        self.plot_button.clicked.connect(self.plot_path)
        self.show_risk_button.clicked.connect(self.show_risk_values)
        self.move_button.clicked.connect(self.move_to_next_state_and_update)

        self.redirector = StdoutRedirector()
        self.redirector.print_signal.connect(self.append_output)
        sys.stdout = self.redirector

    def append_output(self, text):
        self.output_text_edit.append(text.strip())
        self.output_text_edit.verticalScrollBar().setValue(
            self.output_text_edit.verticalScrollBar().maximum()
        )

    def init_ui(self):
        main_layout = QHBoxLayout()

        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        main_layout.addWidget(self.canvas)

        controls_layout = QVBoxLayout()
        controls_layout.setAlignment(Qt.AlignTop)

        input_group = QGroupBox("Params Input")
        input_layout = QFormLayout()
        input_layout.addRow("Time Step:", self.timestep_input)
        input_layout.addRow("Start Index:", self.ref_index_input)
        input_group.setLayout(input_layout)
        controls_layout.addWidget(input_group)

        button_group = QGroupBox("Action")
        button_layout = QVBoxLayout()
        button_layout.addWidget(self.generate_button)
        button_layout.addWidget(self.plot_button)
        button_layout.addWidget(self.show_risk_button)
        button_layout.addWidget(self.move_button)
        button_group.setLayout(button_layout)
        controls_layout.addWidget(button_group)

        risk_group = QGroupBox("Risk Show")
        risk_layout = QVBoxLayout()
        risk_layout.addWidget(self.ego_risk_label)
        risk_layout.addWidget(self.obst_risk_label)
        risk_group.setLayout(risk_layout)
        controls_layout.addWidget(risk_group)

        output_group = QGroupBox("Terminal Output")
        output_layout = QVBoxLayout()
        output_layout.addWidget(self.output_text_edit)
        output_group.setLayout(output_layout)
        controls_layout.addWidget(output_group)

        main_layout.addLayout(controls_layout)
        self.setLayout(main_layout)
        self.setWindowTitle('Planner and V2.0')
        self.show()

    def generate_plot(self):
        self.timestep = int(self.timestep_input.text())
        self.ref_index = int(self.ref_index_input.text())
        self.velocity = None

        self.planner = PlannerINTR(self.file_path, self.timestep)

        self.planner.init_state(self.ref_index, self.velocity)
        self.planner.plan(10)

        self.ego_risk_list, self.obst_risk_list = self.planner.ego_risk_list, self.planner.obst_risk_list

        self.traj_index = int(self.get_min_trajectory())
        self.planner.update_optimal_trajectory(self.traj_index)


    def move_to_next_state_and_update(self):
        self.timestep = int(self.timestep_input.text())
        self.ref_index = int(self.ref_index_input.text())

        self.planner.move_to_next_state_and_update()
        self.planner.plan(10)

        self.ego_risk_list, self.obst_risk_list = self.planner.ego_risk_list, self.planner.obst_risk_list
        self.traj_index = int(self.get_min_trajectory())
        self.planner.update_optimal_trajectory(self.traj_index)


    def plot_path(self):
        self.planner.update_optimal_trajectory(self.traj_index)

        self.figure.clear()

        ax = self.figure.add_subplot(111)
        visualizer = ScenarioVisualization(self.planner.scenario_intr, self.planner.x_0, self.planner.ego_vehicle, ax)
        visualizer.plot_scenario()
        visualizer.plot_optimal_trajectory(self.planner.chosen_trajectory)
        self.canvas.draw()

    def get_min_trajectory(self):
        min_index_ids = find_min_risk_trajectory_for_vehicle(self.veh_id, self.ego_risk_list, self.obst_risk_list)
        max_index_ids = find_max_risk_trajectory_for_vehicle(self.veh_id, self.ego_risk_list, self.obst_risk_list)
        aver_index_ids = find_min_sum_risk_indices(self.ego_risk_list, self.obst_risk_list)
        return aver_index_ids[0]

    def next_trajectory(self):
        self.traj_index = self.traj_index + 1
        self.figure.clear()

        self.planner.update_optimal_trajectory(self.traj_index)

        ax = self.figure.add_subplot(111)
        visualizer = ScenarioVisualization(self.planner.scenario_intr, self.planner.x_0, self.planner.ego_vehicle, ax)
        visualizer.plot_scenario()
        visualizer.plot_optimal_trajectory(self.planner.chosen_trajectory)
        self.canvas.draw()
        self.show_risk_values()

    def show_risk_values(self):
        if self.ego_risk_list is not None and self.obst_risk_list is not None:
            self.ego_risk_label.setText(f"ego_risk: {self.ego_risk_list[self.traj_index]:.4f}")

            obst_risk_str = "obst_risk:\n"
            for obstacle_id, risk_value in self.obst_risk_list[self.traj_index].items():
                obst_risk_str += f"{obstacle_id}: {risk_value:.4f}\n"
            self.obst_risk_label.setText(obst_risk_str)


if __name__ == '__main__':
    import matplotlib
    matplotlib.use('MacOSX')
    app = QApplication(sys.argv)
    # DEU_Flensburg-96_1_T-1
    # DEU_Ibbenbueren-14_1_T-1
    # ESP_Ceuta-1_2_T-1
    main_window = MainWindow("scenarios/ESP_Cambre-3_4_T-1.xml")
    sys.exit(app.exec_())