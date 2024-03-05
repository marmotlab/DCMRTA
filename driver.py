import copy

import torch
import torch.optim as optim
import torch.nn as nn
from torch.utils.tensorboard import SummaryWriter
import ray
import os
import numpy as np
import random
import wandb

from attention import AttentionNet
from runner import RLRunner
from parameters import *
from env.task_env import TaskEnv
from scipy.stats import ttest_rel

ray.init()
writer = SummaryWriter(train_path)
if not os.path.exists(model_path):
    os.makedirs(model_path)
if not os.path.exists(gifs_path):
    os.makedirs(gifs_path)

global_step = None
if WANDB_LOG:
    wandb.init(project="CF")


def writeToTensorBoard(writer, tensorboardData, curr_episode, plotMeans=True):
    # each row in tensorboardData represents an episode
    # each column is a specific metric

    if plotMeans == True:
        tensorboardData = np.array(tensorboardData)
        tensorboardData = list(np.nanmean(tensorboardData, axis=0))
        reward, valueLoss, policyLoss, entropy, gradNorm, success_rate, time, time_cost, waiting, distance, effi = tensorboardData
    else:
        reward, valueLoss, policyLoss, entropy, gradNorm, success_rate, time, time_cost, waiting, distance, effi = tensorboardData

    writer.add_scalar(tag='Losses/Policy Loss', scalar_value=policyLoss, global_step=curr_episode)
    writer.add_scalar(tag='Losses/Entropy', scalar_value=entropy, global_step=curr_episode)
    writer.add_scalar(tag='Losses/Grad Norm', scalar_value=gradNorm, global_step=curr_episode)
    writer.add_scalar(tag='Losses/Value Loss', scalar_value=valueLoss, global_step=curr_episode)

    writer.add_scalar(tag='Perf/Reward', scalar_value=reward, global_step=curr_episode)
    writer.add_scalar(tag='Perf/Makespan', scalar_value=time, global_step=curr_episode)
    writer.add_scalar(tag='Perf/Success rate', scalar_value=success_rate, global_step=curr_episode)
    writer.add_scalar(tag='Perf/Time cost', scalar_value=time_cost, global_step=curr_episode)
    writer.add_scalar(tag='Perf/Waiting time', scalar_value=waiting, global_step=curr_episode)
    writer.add_scalar(tag='Perf/Traveling distance', scalar_value=distance, global_step=curr_episode)
    writer.add_scalar(tag='Perf/Waiting Efficiency', scalar_value=effi, global_step=curr_episode)
    if WANDB_LOG:
        wandb.log({"Losses": {"Grad Norm": gradNorm, "Policy Loss": policyLoss, "Entropy": entropy},
                   "Perf": {"Reward": reward, "Time": time, "Success Rate": success_rate,
                            "Waiting Time": waiting, "Traveling Distance": distance, "Waiting Efficiency": effi}},
                  step=curr_episode)


def main():
    device = torch.device('cuda') if USE_GPU_GLOBAL else torch.device('cpu')
    local_device = torch.device('cuda') if USE_GPU else torch.device('cpu')

    global_network = AttentionNet(AGENT_INPUT_DIM, TASK_INPUT_DIM, EMBEDDING_DIM).to(device)
    baseline_network = AttentionNet(AGENT_INPUT_DIM, TASK_INPUT_DIM, EMBEDDING_DIM).to(device)
    global_optimizer = optim.Adam(global_network.parameters(), lr=LR)
    lr_decay = optim.lr_scheduler.StepLR(global_optimizer, step_size=DECAY_STEP, gamma=0.98)
    # Automatically logs gradients of pytorch model
    if WANDB_LOG:
        wandb.watch(global_network)

    curr_episode = 0
    best_perf = -100
    curr_level = 0
    if LOAD_MODEL:
        print('Loading Model...')
        checkpoint = torch.load(model_path + '/checkpoint.pth')
        global_network.load_state_dict(checkpoint['model'])
        baseline_network.load_state_dict(checkpoint['model'])
        global_optimizer.load_state_dict(checkpoint['optimizer'])
        lr_decay.load_state_dict(checkpoint['lr_decay'])
        curr_episode = checkpoint['episode']
        curr_level = checkpoint['level']
        print("curr_episode set to ", curr_episode)

        if os.path.exists(model_path + '/best_model_checkpoint.pth'):
            best_model_checkpoint = torch.load(model_path + '/best_model_checkpoint.pth')
            best_perf = best_model_checkpoint['best_perf']
            baseline_network.load_state_dict(best_model_checkpoint['model'])
            print('best performance so far:', best_perf)
        print(global_optimizer.state_dict()['param_groups'][0]['lr'])
        if RESET_OPT:
            global_optimizer = optim.Adam(global_network.parameters(), lr=LR)
            lr_decay = optim.lr_scheduler.StepLR(global_optimizer, step_size=DECAY_STEP, gamma=0.98)
            curr_episode = 0

    # launch meta agents
    meta_agents = [RLRunner.remote(i) for i in range(NUM_META_AGENT)]

    # get initial weights
    if device != local_device:
        weights = global_network.to(local_device).state_dict()
        baseline_weights = baseline_network.to(local_device).state_dict()
        global_network.to(device)
        baseline_network.to(device)
    else:
        weights = global_network.state_dict()
        baseline_weights = baseline_network.state_dict()

    # launch the first job on each runner

    jobList = []
    agents_num = np.random.randint(AGENTS_RANGE[0], AGENTS_RANGE[1] + 1)
    tasks_num = np.random.randint(TASKS_RANGE[0], TASKS_RANGE[1] + 1)
    for i, meta_agent in enumerate(meta_agents):
        jobList.append(meta_agent.job.remote(weights, baseline_weights, curr_episode, agents_num, tasks_num))
        curr_episode += 1
    metric_name = ['success_rate', 'makespan', 'time_cost', 'waiting_time', 'travel_dist', 'efficiency']
    tensorboardData = []
    trainingData = []
    experience_buffer = [[] for _ in range(9)]
    test_set = np.random.randint(low=0, high=1e8, size=[256 // NUM_META_AGENT, NUM_META_AGENT])
    baseline_value = None

    try:
        while True:
            # wait for any job to be completed
            done_id, jobList = ray.wait(jobList, num_returns=NUM_META_AGENT)
            done_jobs = ray.get(done_id)
            random.shuffle(done_jobs)
            perf_metrics = {}
            for n in metric_name:
                perf_metrics[n] = []
            for job in done_jobs:
                jobResults, metrics, info = job
                for i in range(9):
                    experience_buffer[i] += jobResults[i]
                for n in metric_name:
                    perf_metrics[n].append(metrics[n])

            update_done = False
            while len(experience_buffer[0]) >= BATCH_SIZE:
                agents_num = np.random.randint(AGENTS_RANGE[0], AGENTS_RANGE[1] + 1)
                tasks_num = np.random.randint(TASKS_RANGE[0], TASKS_RANGE[1] + 1)
                rollouts = copy.copy(experience_buffer)
                for i in range(len(rollouts)):
                    rollouts[i] = rollouts[i][:BATCH_SIZE]
                for i in range(len(experience_buffer)):
                    experience_buffer[i] = experience_buffer[i][BATCH_SIZE:]
                if len(experience_buffer[0]) < BATCH_SIZE:
                    update_done = True
                if update_done:
                    experience_buffer = []
                    for i in range(9):
                        experience_buffer.append([])

                agent_inputs = torch.stack(rollouts[0], dim=0)  # (batch,sample_size,2)
                task_inputs = torch.stack(rollouts[1], dim=0)  # (batch,sample_size,k_size)
                action_batch = torch.stack(rollouts[2], dim=0)  # (batch,1,1)
                mask_batch = torch.stack(rollouts[3], dim=0)  # (batch,1,1)
                advantage_batch = torch.stack(rollouts[6], dim=0)  # (batch,1,1)
                reward_batch = torch.stack(rollouts[4], dim=0)  # (batch,1,1)
                index = torch.stack(rollouts[5])

                if device != local_device:
                    agent_inputs = agent_inputs.to(device)
                    task_inputs = task_inputs.to(device)
                    action_batch = action_batch.to(device)
                    mask_batch = mask_batch.to(device)
                    reward_batch = reward_batch.to(device)
                    advantage_batch = advantage_batch.to(device)
                    index = index.to(device)

                logp_list = global_network(task_inputs, agent_inputs, mask_batch) #, lstm_c, lstm_h)
                logp = torch.gather(logp_list, 1, action_batch)
                entropy = (logp_list * logp_list.exp()).nansum(dim=-1).mean()
                policy_loss = - logp * advantage_batch.detach()
                policy_loss = policy_loss.mean()


                loss = policy_loss
                global_optimizer.zero_grad()

                loss.backward()
                grad_norm = torch.nn.utils.clip_grad_norm_(global_network.parameters(), max_norm=10, norm_type=2)
                global_optimizer.step()
                lr_decay.step()

                perf_data = []
                for n in metric_name:
                    perf_data.append(np.nanmean(perf_metrics[n]))
                data = [reward_batch.mean().item(), 0, policy_loss.item(),
                        entropy.item(), grad_norm.item(), *perf_data]
                trainingData.append(data)

            for i, meta_agent in enumerate(meta_agents):
                jobList.append(meta_agent.job.remote(weights, baseline_weights, curr_episode, agents_num, tasks_num))
                curr_episode += 1

            if len(trainingData) >= SUMMARY_WINDOW:
                writeToTensorBoard(writer, trainingData, curr_episode)
                trainingData = []

            # get the updated global weights
            if update_done:
                if device != local_device:
                    weights = global_network.to(local_device).state_dict()
                    baseline_weights = baseline_network.to(local_device).state_dict()
                    global_network.to(device)
                    baseline_network.to(device)
                else:
                    weights = global_network.state_dict()
                    baseline_weights = baseline_network.state_dict()

            if curr_episode % 512 == 0:
                print('Saving model', end='\n')
                checkpoint = {"model": global_network.state_dict(),
                              "optimizer": global_optimizer.state_dict(),
                              "episode": curr_episode,
                              "lr_decay": lr_decay.state_dict(),
                              "level": curr_level,
                              "best_perf": best_perf
                }
                path_checkpoint = "./" + model_path + "/checkpoint.pth"
                torch.save(checkpoint, path_checkpoint)
                print('Saved model', end='\n')


            if EVALUATE:
                if curr_episode % 1024 == 0:
                    # stop the training
                    ray.wait(jobList, num_returns=NUM_META_AGENT)
                    for a in meta_agents:
                        ray.kill(a)
                    torch.cuda.empty_cache()
                    print('Evaluate baseline model at ', curr_episode)

                    # test the baseline model on the new test set
                    if baseline_value is None:
                        test_agent_list = [RLRunner.remote(metaAgentID=i) for i in range(NUM_META_AGENT)]
                        for _, test_agent in enumerate(test_agent_list):
                            ray.get(test_agent.set_baseline_weights.remote(baseline_weights))
                        rewards = []
                        for i in range(256 // NUM_META_AGENT):
                            sample_job_list = []
                            for j, test_agent in enumerate(test_agent_list):
                                sample_job_list.append(test_agent.testing.remote(seed=test_set[i][j]))
                            sample_done_id, _ = ray.wait(sample_job_list, num_returns=NUM_META_AGENT)
                            reward = ray.get(sample_done_id)
                            rewards = rewards + reward
                        baseline_value = np.stack(rewards)
                        for a in test_agent_list:
                            ray.kill(a)

                    # test the current model's performance
                    test_agent_list = [RLRunner.remote(metaAgentID=i) for i in range(NUM_META_AGENT)]
                    for _, test_agent in enumerate(test_agent_list):
                        ray.get(test_agent.set_baseline_weights.remote(weights))
                    rewards = []
                    for i in range(256 // NUM_META_AGENT):
                        sample_job_list = []
                        for j, test_agent in enumerate(test_agent_list):
                            sample_job_list.append(test_agent.testing.remote(seed=test_set[i][j]))
                        sample_done_id, _ = ray.wait(sample_job_list, num_returns=NUM_META_AGENT)
                        reward = ray.get(sample_done_id)
                        rewards = rewards + reward
                    test_value = np.stack(rewards)
                    for a in test_agent_list:
                        ray.kill(a)

                    meta_agents = [RLRunner.remote(i) for i in range(NUM_META_AGENT)]

                    # update baseline if the model improved more than 5%
                    print('test value', test_value.mean())
                    print('baseline value', baseline_value.mean())
                    if test_value.mean() > baseline_value.mean():
                        _, p = ttest_rel(test_value, baseline_value)
                        print('p value', p)
                        if p < 0.05:
                            print('update baseline model at ', curr_episode)
                            if device != local_device:
                                weights = global_network.to(local_device).state_dict()
                                global_network.to(device)
                            else:
                                weights = global_network.state_dict()
                            baseline_weights = copy.deepcopy(weights)
                            baseline_network.load_state_dict(baseline_weights)
                            test_set = np.random.randint(low=0, high=1e8, size=[256 // NUM_META_AGENT, NUM_META_AGENT])
                            print('update test set')
                            baseline_value = None
                            best_perf = test_value.mean()
                            print('Saving best model', end='\n')
                            checkpoint = {"model": global_network.state_dict(),
                                          "optimizer": global_optimizer.state_dict(),
                                          "episode": curr_episode,
                                          "lr_decay": lr_decay.state_dict(),
                                          "best_perf": best_perf}
                            path_checkpoint = "./" + model_path + "/best_model_checkpoint.pth"
                            torch.save(checkpoint, path_checkpoint)
                            print('Saved model', end='\n')
                    jobList = []
                    for i, meta_agent in enumerate(meta_agents):
                        jobList.append(meta_agent.job.remote(weights, baseline_weights, curr_episode, agents_num, tasks_num))
                        curr_episode += 1

    except KeyboardInterrupt:
        print("CTRL_C pressed. Killing remote workers")
        if WANDB_LOG:
            wandb.finish()
        for a in meta_agents:
            ray.kill(a)


if __name__ == "__main__":
    main()
