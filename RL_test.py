import os
import torch
from attention import AttentionNet
from worker import Worker
import numpy as np
from env.task_env import TaskEnv
import time
import pickle
import pandas as pd

INPUT_DIM = 3
EMBEDDING_DIM = 128
AGENT_INPUT_DIM = 6
TASK_INPUT_DIM = 5
USE_GPU = False
USE_GPU_GLOBAL = True
NUM_GPU = 0
NUM_META_AGENT = 1
GAMMA = 1
FOLDER_NAME = 'REINFORCE'
METHOD = 'LF' # or 'IA'
testSet = 'testSet_20A_50T_CONDET'
model_path = f'model/{FOLDER_NAME}'

device = torch.device('cuda:0') if USE_GPU_GLOBAL else torch.device('cpu')
local_device = torch.device('cuda:0') if USE_GPU else torch.device('cpu')
global_network = AttentionNet(AGENT_INPUT_DIM, TASK_INPUT_DIM, EMBEDDING_DIM).to(device)
checkpoint = torch.load(f'{model_path}/checkpoint.pth', map_location=torch.device('cpu'))
global_network.load_state_dict(checkpoint['model'])
worker = Worker(0, global_network, global_network, 0, device)
perf_metrics = {'success_rate': [], 'makespan': [], 'time_cost':[], 'waiting_time': [], 'travel_dist': [], 'efficiency': []}
df = pd.DataFrame(perf_metrics)

for i in range(0, 50):
    env = pickle.load(open(f'{testSet}/env_{i}.pkl', 'rb'))
    agents = env.agent_dic
    tasks = env.task_dic
    depot = env.depot
    env.max_waiting_time = 10
    env.reactive_planning = False
    test_env = (tasks, agents, depot)
    env.reset(test_env)
    env.clear_decisions()
    env.force_waiting = True
    if METHOD == 'IA':
        results = worker.run_test_IS(0, env)  # for LF
    else:
        results = worker.run_test(0, env)  # for IS
    df_ = pd.DataFrame(results, index=[i])
    df = pd.concat([df, df_])
df.to_csv(f'{testSet}/REINFORCE_{METHOD}.csv')

