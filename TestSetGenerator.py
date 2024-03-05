import os.path
import random
from env.task_env import TaskEnv
import yaml
import math
from itertools import permutations
import pickle
import os

test_set = 'testSet_20A_50T_CONDET'
test_instances_num = 50
agents_range = (20, 20)
tasks_range = (50, 50)
if not os.path.exists(f'./{test_set}'):
    os.makedirs(f'./{test_set}')
    for i in range(test_instances_num):
        env = TaskEnv(agents_range, tasks_range, traits_dim=1, max_coalition_size=5, seed=i)
        pickle.dump(env, open(f'../{test_set}/env_{i}.pkl', 'wb'))
agent_yaml = dict()
task_yaml = dict()
planner_param = dict()
graph_yaml = dict()
folder = test_set
planner = 'TEAMPLANNER_CONDET'
solver_time = 300.0
def compute_euclidean_distance_matrix(locations):
    """Creates callback to return distance between points."""
    distances = {}
    for from_counter, from_node in enumerate(locations):
        distances[from_counter] = {}
        for to_counter, to_node in enumerate(locations):
            if from_counter == to_counter:
                distances[from_counter][to_counter] = 0
            else:
                # Euclidean distance
                distances[from_counter][to_counter] = math.hypot((from_node[0] - to_node[0]),(from_node[1] - to_node[1]))
    return distances


for i in range(test_instances_num):
    env = pickle.load(open(f'{folder}/env_{i}.pkl', 'rb'))
    if os.path.exists(f'{folder}/env_{i}.pkl'):
        if not os.path.exists(f'{folder}/env_{i}'):
            os.mkdir(f'{folder}/env_{i}')
        if os.path.exists(f'{folder}/env_{i}/*.yaml'):
            continue
    env.force_waiting = True
    coords = env.get_matrix(env.task_dic, 'location')
    dist_matrix = compute_euclidean_distance_matrix(coords)
    depot = env.depot['location']
    depot_distance = [math.hypot((depot[0] - coords[i][0]),(depot[1] - coords[i][1])) for i in range(len(coords))]
    p = list(permutations(range(len(env.task_dic)), 2))
    if planner == 'TEAMPLANNER_CONDET':
        agent_yaml.update({f'vehicle0': {'engCap': 1e6, 'engCost': 0., 'capVector': [1.0], 'capVar': [0.]}})
        graph_yaml.update({f'vehicle0': {
            f'edge{i}': [t[0], t[1], 0, dist_matrix[t[0]][t[1]], 0, float(dist_matrix[t[0]][t[1]]/0.2)] for i, t in enumerate(p)
        }
        })
        for j in range(len(env.task_dic)):
            graph_yaml[f'vehicle0'][f'edge{2*j+len(p)}'] = [int(env.tasks_num), j, 0, depot_distance[j], 0, depot_distance[j]/0.2]
            graph_yaml[f'vehicle0'][f'edge{2*j+len(p)+1}'] = [j, int(env.tasks_num) + 1, 0, depot_distance[j], 0, depot_distance[j]/0.2]
        for id, task in env.task_dic.items():
            graph_yaml[f'vehicle0'][f'node{id}'] = float(task['time'])
    elif planner == 'TEAMPLANNER_DET':
        for a in range(len(env.agent_dic)):
            agent_yaml.update({f'vehicle{a}': {'engCap': 1e6, 'engCost': 1., 'capVector': [1.0], 'capVar': [0.]}})
            graph_yaml.update({f'vehicle{a}': {
                f'edge{i}': [t[0], t[1], 0, dist_matrix[t[0]][t[1]], 0, float(dist_matrix[t[0]][t[1]]/0.2)] for i, t in enumerate(p)
            }
            })
            for j in range(len(env.task_dic)):
                graph_yaml[f'vehicle{a}'][f'edge{2*j+len(p)}'] = [int(env.tasks_num) + a, j, 0, depot_distance[j], 0, depot_distance[j]/0.2]
                graph_yaml[f'vehicle{a}'][f'edge{2*j+len(p)+1}'] = [j, int(env.tasks_num) + int(env.agents_num) + a, 0, depot_distance[j], 0, depot_distance[j]/0.2]
            for id, task in env.task_dic.items():
                graph_yaml[f'vehicle{a}'][f'node{id}'] = float(task['time'])
    # [from node, to node, the first edge between the two nodes energy cost, variance of the energy cost, time cost]
    for task in env.task_dic.items():
        task_yaml.update({f'task{task[0]}': {'and0': {'or0':{'geq': True, 'capId': 0, 'capReq': float(task[1]['requirements']), 'capVar': 0.}}}})
    with open(f'{folder}/env_{i}/vehicle_param.yaml', 'w') as f:
        yaml.dump(agent_yaml, f, sort_keys=False)
    with open(f'{folder}/env_{i}/task_param.yaml', 'w') as f:
        yaml.dump(task_yaml, f, sort_keys=False)
    planner_param = {
                    'flagOptimizeCost': True,
                    'flagTaskComplete': True,
                    'flagSprAddCutToSameType': True,
                    'taskCompleteReward': 10000,
                    'timePenalty': 100,
                    'recoursePenalty': 1.0,
                    'taskRiskPenalty': 0.0,
                    'LARGETIME': 10000.0,
                    'MAXTIME': 1000.0,
                    'MAXENG': 1E8,
                    'flagSolver': planner,
                    'CcpBeta': 0.95,
                    'taskBeta': 0.95,
                    'solverMaxTime': solver_time,
                    'solverIterMaxTime': 50.0,
                    'flagNotUseUnralavant': True,
                    'MAXALPHA': 20.0,
                    'taskNum': int(env.tasks_num),
                    'vehNum': 1 if planner == 'TEAMPLANNER_CONDET' else int(env.agents_num),
                    'capNum': 1,
                    'vehTypeNum': 1,
                    'vehNumPerType': [int(env.agents_num)] if planner == 'TEAMPLANNER_CONDET' else [1] * int(env.agents_num),
                    'sampleNum': 500,
                    'randomType': 0,
                    'capType': [0],
                    'vehicleParamFile': f'./{folder}/env_{i}/vehicle_param.yaml',
                    'taskParamFile': f'./{folder}/env_{i}/task_param.yaml',
                    'graphFile': f'./{folder}/env_{i}/graph.yaml'
    }
    with open(f'{folder}/env_{i}/planner_param.yaml', 'w') as f:
        yaml.dump(planner_param, f, sort_keys=False)
    with open(f'{folder}/env_{i}/graph.yaml', 'w') as f:
        yaml.dump(graph_yaml, f, sort_keys=False)
