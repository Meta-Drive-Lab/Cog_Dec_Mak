from commonroad_route_planner.route_planner import RoutePlanner

from utils_coordinate_system import extend_ref_path_both_ends, smooth_ref_path


def get_ref_path(scenario, planning_problem):
    route_planner = RoutePlanner(lanelet_network=scenario.lanelet_network,
                                 planning_problem=planning_problem,
                                 scenario=scenario,
                                 extended_search=False)
    shortest_route = route_planner.plan_routes().retrieve_shortest_route(retrieve_shortest=True)
    reference_path = extend_ref_path_both_ends(shortest_route.reference_path)
    reference_path = smooth_ref_path(reference_path)
    return reference_path