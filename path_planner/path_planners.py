from utils.configurations import PathPlannerType


def get_path_planner(configs):
    if configs.type == PathPlannerType.a_star:
        return None
    elif configs.type == PathPlannerType.rrt_star:
        return None
    else:
        raise ValueError("the path planner type {} is not defined".format(configs.type))