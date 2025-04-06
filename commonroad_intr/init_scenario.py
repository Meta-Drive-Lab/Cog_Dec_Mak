from commonroad.common.file_reader import CommonRoadFileReader

from get_ref_path import get_ref_path
from planner_params import define_visual_params
from utils_coordinate_system import CoordinateSystem


def init_scenario(file_path):
    scenario, planning_problem_set = CommonRoadFileReader(file_path).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    return scenario, planning_problem


class ScenarioINTR:
    def __init__(self, file_path):
        self.obs_params = None
        self.ego_params = None
        self.timestep = None
        self.scenario, self.planning_problem = init_scenario(file_path)

        self.reference_path = get_ref_path(self.scenario, self.planning_problem)
        self.coordinate_system = CoordinateSystem(reference=self.reference_path)
        
    def set_scenario_timestep(self, timestep):
        self.timestep = timestep
        
    def set_scenario_params(self):
        if self.timestep is None:
            self.ego_params, self.obs_params = define_visual_params(timestep=0)
        else:
            self.ego_params, self.obs_params = define_visual_params(timestep=self.timestep)

    def update_scenario(self, timestep):
        self.set_scenario_timestep(timestep)
        self.set_scenario_params()