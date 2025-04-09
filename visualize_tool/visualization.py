import numpy as np
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.visualization.mp_renderer import MPRenderer
from matplotlib import pyplot as plt

from init_scenario import ScenarioINTR
from state import ReactivePlannerState


class ScenarioVisualization:
    def __init__(self, scenario_intr: ScenarioINTR, x_0: ReactivePlannerState, ego_vehicle: DynamicObstacle, ax_func):
        self.scenario = scenario_intr
        self.x_0 = x_0
        self.ego_vehicle = ego_vehicle
        self.ax = ax_func

    def plot_scenario(self):

        ego_start_pos = self.x_0.position
        plot_limits = [-30 + ego_start_pos[0], 30 + ego_start_pos[0],
                       -30 + ego_start_pos[1], 30 + ego_start_pos[1]]
        rnd = MPRenderer(plot_limits=plot_limits, ax=self.ax, figsize=(7, 7))

        ego_params, obs_params = self.scenario.ego_params, self.scenario.obs_params
        self.scenario.scenario.draw(rnd, draw_params=obs_params)
        self.ego_vehicle.draw(rnd, draw_params=ego_params)
        rnd.render()

        plt.plot(self.scenario.reference_path[:, 0], self.scenario.reference_path[:, 1], color='blue', linewidth=2.0, picker=False, zorder=18)

    def plot_optimal_trajectory(self, chosen_trajectory):

        if len(chosen_trajectory.state_list) > 0:
            optimal_traj_positions = np.array(
                [(state.position[0], state.position[1]) for state in chosen_trajectory.state_list[0:]])
            self.ax.plot(optimal_traj_positions[:, 0], optimal_traj_positions[:, 1], color='green', linewidth=2.0,
                         zorder=25)
