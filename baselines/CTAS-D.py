import copy
import os.path

from env.task_env import TaskEnv
import math
import numpy as np
import yaml


def get_agent_route(param_file_pth, result_file_pth):
    yaml_result_file = result_file_pth
    yaml_param_file = param_file_pth

    with open(yaml_param_file, 'r') as f:
        param_data = yaml.safe_load(f)
    if param_data['flagSolver'] == 'TEAMPLANNER_DET':
        num_veh = param_data['vehNum']
    else:
        num_veh = param_data['vehNumPerType'][0]
    with open(yaml_result_file, 'r') as f:
        data = yaml.safe_load(f)

    if 'vehicle' not in data:
        return None
    # Access the nested field
    nodes = []
    for i in range(num_veh):
        if 'vv' + str(i + 1) not in data['vehicle']:
            continue
        node_visited = data['vehicle']['vv' + str(i + 1)]['node']
        nodes.append(node_visited)

    return nodes


def CTASD_read_results(env, path):
    if os.path.exists(path + 'results.yaml'):
        routes = get_agent_route(path + 'planner_param.yaml', path + 'results.yaml')
        if routes is None:
            return None
        for i in range(len(routes)):
            if routes[i] == [0]:
                continue
            else:
                env.pre_set_route(copy.copy(routes[i])[1:], i)
        return True


if __name__ == '__main__':
    import pickle
    import pandas as pd
    import glob
    from natsort import natsorted
    time = []
    folder = 'testSet_20A_50T_CONDET'
    method = 'CTAS-D'
    files = natsorted(glob.glob(f'./{folder}/env_*.pkl'), key=lambda y: y.lower())
    perf_metrics = {'success_rate':[], 'makespan': [], 'time_cost':[], 'waiting_time': [], 'travel_dist': [], 'efficiency': []}
    for i in files:
        env = pickle.load(open(i, 'rb'))
        agents = env.agent_dic
        tasks = env.task_dic
        depot = env.depot
        env.reactive_planning = False
        test_env = (tasks, agents, depot)
        env.reset(test_env)
        env.clear_decisions()
        if method == 'CTAS-D':
            re = CTASD_read_results(env, i.replace('.pkl', '/'))
            if re is None:
                perf_metrics['success_rate'].append(0)
                perf_metrics['makespan'].append(np.nan)
                perf_metrics['time_cost'].append(np.nan)
                perf_metrics['waiting_time'].append(np.nan)
                perf_metrics['travel_dist'].append(np.nan)
                perf_metrics['efficiency'].append(np.nan)
                continue
        env.force_wait = True
        env.execute_by_route(i.replace('.pkl', '/'), method, False)
        reward, finished_tasks = env.get_episode_reward(100)
        if np.sum(finished_tasks) / len(finished_tasks) < 1:
            perf_metrics['success_rate'].append(np.sum(finished_tasks) / len(finished_tasks))
            perf_metrics['makespan'].append(np.nan)
            perf_metrics['time_cost'].append(np.nan)
            perf_metrics['waiting_time'].append(np.nan)
            perf_metrics['travel_dist'].append(np.nan)
            perf_metrics['efficiency'].append(np.nan)
        else:
            perf_metrics['success_rate'].append(np.sum(finished_tasks) / len(finished_tasks))
            perf_metrics['makespan'].append(env.current_time)
            perf_metrics['time_cost'].append(np.sum(np.nan_to_num(env.get_matrix(env.task_dic, 'time_start'), nan=100)))
            perf_metrics['waiting_time'].append(np.mean(env.get_matrix(env.agent_dic, 'sum_waiting_time')))
            perf_metrics['travel_dist'].append(np.sum(env.get_matrix(env.agent_dic, 'travel_dist')))
            perf_metrics['efficiency'].append(np.mean(env.get_matrix(env.task_dic, 'sum_waiting_time')))
        print(i)
    df = pd.DataFrame(perf_metrics)
    df.to_csv(f'./{folder}/{method}.csv')