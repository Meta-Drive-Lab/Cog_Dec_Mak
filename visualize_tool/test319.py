from matplotlib import pyplot as plt

from planner_visual_tool import PlannerINTR
from visualization import ScenarioVisualization


def plot_and_show(file_path_func, ref_index_func, timestep_func, traj_index_func, ax_func):
    planner = PlannerINTR(file_path_func, timestep_func)
    planner.init_state(ref_index_func)
    planner.plan(traj_index_func)


    visualizer = ScenarioVisualization(planner.scenario_intr, planner.x_0, planner.ego_vehicle, ax_func)
    visualizer.plot_scenario()
    visualizer.plot_optimal_trajectory(planner.chosen_trajectory)
    plt.draw()  # 仅更新图像，不关闭窗口

    planner.move_to_next_state_and_update()
    planner.plan(traj_index_func)

    visualizer = ScenarioVisualization(planner.scenario_intr, planner.x_0, planner.ego_vehicle, ax_func)
    visualizer.plot_scenario()
    visualizer.plot_optimal_trajectory(planner.chosen_trajectory)
    plt.draw()  # 仅更新图像，不关闭窗口




if __name__ == '__main__':
    file_path = "scenarios/ESP_Cambre-3_4_T-1.xml"
    timestep = 15
    ref_index = 60
    fig, ax = plt.subplots()
    plot_and_show(file_path, ref_index, timestep, 77, ax)
    plt.show()  # 在主程序里调用一次，避免多次关闭