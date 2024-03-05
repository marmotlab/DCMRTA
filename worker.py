import pickle
import time
import torch
import numpy as np
from env.task_env import TaskEnv
from attention import AttentionNet
import scipy.signal as signal
from parameters import *
import copy
from torch.nn import functional as F
from torch.distributions import Categorical


def discount(x, gamma):
    return signal.lfilter([1], [1, -gamma], x[::-1], axis=0)[::-1]


def zero_padding(x, padding_size, length):
    pad = torch.nn.ZeroPad2d((0, 0, 0, padding_size - length))
    x = pad(x)
    return x


class Worker:
    def __init__(self, mete_agent_id, local_network, local_baseline, global_step,
                 device='cuda', save_image=False, agents_num=AGENTS_RANGE, tasks_num=TASKS_RANGE, seed=None):

        self.device = device
        self.metaAgentID = mete_agent_id
        self.global_step = global_step
        self.save_image = save_image
        self.env = TaskEnv(agents_num, tasks_num, TRAIT_DIM, COALITION_SIZE, seed=seed, plot_figure=save_image)
        self.baseline_env = copy.deepcopy(self.env)
        self.local_net = local_network
        self.local_baseline = local_baseline
        self.experience = None
        self.episode_number = None
        self.perf_metrics = {}
        self.p_rnn_state = {}

    def run_episode(self, episode_number):
        episode_buffer = [[] for _ in range(9)]
        perf_metrics = {}
        current_action_index = 0
        while not self.env.finished and self.env.current_time < MAX_TIME:
            with torch.no_grad():
                decision_agents, current_time = self.env.next_decision()
                groups = self.env.get_unique_group(decision_agents)
                self.env.current_time = current_time
                self.env.task_update()
                self.env.agent_update()
                for group in groups:
                    while len(group) > 0:
                        leader_id = np.random.choice(group)
                        agent = self.env.agent_dic[leader_id]
                        if not agent['returned']:
                            mask = self.env.get_unfinished_task_mask()
                            if np.sum(mask) == self.env.tasks_num:
                                mask = np.insert(mask, 0, False)
                            else:
                                mask = np.insert(mask, 0, True)
                            total_agents = torch.FloatTensor(self.env.get_current_agent_status(agent)).unsqueeze(0).to(self.device)
                            # total_agents = self.zero_padding(total_agents, AGENTS_RANGE[1])
                            task_info = torch.FloatTensor(self.env.get_current_task_status(agent)).unsqueeze(0).to(self.device)
                            # task_info = self.zero_padding(task_info, TASKS_RANGE[1] + 1)
                            mask = torch.tensor(mask).unsqueeze(0).to(self.device)
                            # mask = self.true_padding(mask, TASKS_RANGE[1] + 1)
                            agent_id = torch.tensor([[[agent['ID']]]]).to(self.device)
                            logp_list = self.local_net(task_info, total_agents, mask)
                            action = Categorical(logp_list.exp()).sample()
                            while action.item() > self.env.tasks_num:
                                action = Categorical(logp_list.exp()).sample()
                            group, r = self.env.step(group, leader_id, action.item(), current_action_index)
                            f = self.env.task_update()
                            agent['current_action_index'] = current_action_index
                            self.env.agent_update()
                            episode_buffer[0] += total_agents
                            episode_buffer[1] += task_info
                            episode_buffer[2] += action.unsqueeze(0)
                            episode_buffer[3] += mask
                            episode_buffer[4] += torch.FloatTensor([[0]]).to(self.device)  # reward
                            episode_buffer[5] += agent_id
                            episode_buffer[6] += torch.FloatTensor([[0]]).to(self.device)  # adv
                            current_action_index += 1
                self.env.finished = self.env.check_finished()

        reward, finished_tasks = self.env.get_episode_reward(MAX_TIME)
        # traceback_rewards = self.env.get_traceback_reward()
        self.baseline_test()
        greedy_reward, _ = self.baseline_env.get_episode_reward(MAX_TIME)
        episode_buffer[4][-1] += reward
        adv = reward - greedy_reward
        episode_buffer[6][-1] += adv
        advantages = copy.deepcopy(episode_buffer[6])

        for i in range(len(advantages)):
            advantages[i] = advantages[i].cpu().numpy()
        discounted_advantages = discount(np.array(advantages).reshape(-1), GAMMA).tolist()
        discounted_advantages = torch.FloatTensor(discounted_advantages).unsqueeze(1).to(self.device)
        for i in range(len(advantages)):
            episode_buffer[6][i] = discounted_advantages[i, :]

        perf_metrics['success_rate'] = np.sum(finished_tasks)/len(finished_tasks)
        perf_metrics['makespan'] = self.env.current_time
        perf_metrics['time_cost'] = np.nanmean(self.env.get_matrix(self.env.task_dic, 'time_start'))
        perf_metrics['waiting_time'] = np.mean(self.env.get_matrix(self.env.agent_dic, 'sum_waiting_time'))
        perf_metrics['travel_dist'] = np.sum(self.env.get_matrix(self.env.agent_dic, 'travel_dist'))
        perf_metrics['efficiency'] = np.mean(self.env.get_matrix(self.env.task_dic, 'sum_waiting_time'))
        if self.save_image:
            self.env.plot_animation(gifs_path, episode_number)
        self.experience = episode_buffer
        return perf_metrics

    def run_test(self, test_episode, test_env, image_path=None):
        perf_metrics = dict()
        self.baseline_env = copy.copy(test_env)
        self.baseline_env.plot_figure = False
        while not self.baseline_env.finished and self.baseline_env.current_time < MAX_TIME:
            with torch.no_grad():
                decision_agents, current_time = self.baseline_env.next_decision()
                groups = self.baseline_env.get_unique_group(decision_agents)
                self.baseline_env.current_time = current_time
                self.baseline_env.task_update()
                self.baseline_env.agent_update()
                for group in groups:
                    while len(group) > 0:
                        leader_id = np.random.choice(group)
                        agent = self.baseline_env.agent_dic[leader_id]
                        if not agent['returned']:
                            mask = self.baseline_env.get_unfinished_task_mask()
                            if np.sum(mask) == self.baseline_env.tasks_num:
                                mask = np.insert(mask, 0, False)
                            else:
                                mask = np.insert(mask, 0, True)
                            total_agents = torch.FloatTensor(self.baseline_env.get_current_agent_status(agent)).unsqueeze(0).to(self.device)
                            task_info = torch.FloatTensor(self.baseline_env.get_current_task_status(agent)).unsqueeze(0).to(self.device)
                            mask = torch.tensor(mask).unsqueeze(0).to(self.device)
                            agent_id = torch.tensor([[[agent['ID']]]]).to(self.device)
                            logp_list = self.local_net(task_info, total_agents, mask)
                            action = torch.argmax(logp_list.exp() * ~mask, dim=1)
                            group, r = self.baseline_env.step(group, leader_id, action.item(), 0)
                            self.baseline_env.task_update()
                            self.baseline_env.agent_update()
                self.baseline_env.finished = self.baseline_env.check_finished()
        reward, finished_tasks = self.baseline_env.get_episode_reward(MAX_TIME)

        perf_metrics['success_rate'] = np.sum(finished_tasks)/len(finished_tasks)
        perf_metrics['makespan'] = self.baseline_env.current_time
        perf_metrics['time_cost'] = np.nanmean(self.baseline_env.get_matrix(self.baseline_env.task_dic, 'time_start'))
        perf_metrics['waiting_time'] = np.mean(self.baseline_env.get_matrix(self.baseline_env.agent_dic, 'sum_waiting_time'))
        perf_metrics['travel_dist'] = np.sum(self.baseline_env.get_matrix(self.baseline_env.agent_dic, 'travel_dist'))
        perf_metrics['efficiency'] = np.mean(self.baseline_env.get_matrix(self.baseline_env.task_dic, 'sum_waiting_time'))
        if image_path is not None:
            self.baseline_env.plot_animation(image_path, 'RL')
        # self.generate_route()
        # self.baseline_env.process_map(image_path)
        return perf_metrics

    def run_test_IS(self, test_episode, test_env):
        perf_metrics = dict()
        self.baseline_env = copy.copy(test_env)
        self.baseline_env.plot_figure = False
        while not self.baseline_env.finished and self.baseline_env.current_time < MAX_TIME:
            with torch.no_grad():
                decision_agents, current_time = self.baseline_env.next_decision()
                # groups = self.baseline_env.get_unique_group(decision_agents)
                self.baseline_env.current_time = current_time
                self.baseline_env.task_update()
                self.baseline_env.agent_update()
                for agent_id in decision_agents:
                    # while len(group) > 0:
                    # leader_id = np.random.choice(group)
                    agent = self.baseline_env.agent_dic[agent_id]
                    if not agent['returned']:
                        mask = self.baseline_env.get_unfinished_task_mask()
                        if np.sum(mask) == self.baseline_env.tasks_num:
                            mask = np.insert(mask, 0, False)
                        else:
                            mask = np.insert(mask, 0, True)
                        total_agents = torch.FloatTensor(self.baseline_env.get_current_agent_status(agent)).unsqueeze(0).to(self.device)
                        task_info = torch.FloatTensor(self.baseline_env.get_current_task_status(agent)).unsqueeze(0).to(self.device)
                        mask = torch.tensor(mask).unsqueeze(0).to(self.device)
                        agent_id = torch.tensor([[[agent['ID']]]]).to(self.device)
                        logp_list = self.local_net(task_info, total_agents, mask)
                        action = torch.argmax(logp_list.exp() * ~mask, dim=1)
                        self.baseline_env.agent_step(agent_id.item(), action.item())
                        self.baseline_env.task_update()
                        self.baseline_env.agent_update()
                self.baseline_env.finished = self.baseline_env.check_finished()
        reward, finished_tasks = self.baseline_env.get_episode_reward(MAX_TIME)

        perf_metrics['success_rate'] = np.sum(finished_tasks)/len(finished_tasks)
        perf_metrics['makespan'] = self.baseline_env.current_time
        perf_metrics['time_cost'] = np.nanmean(self.baseline_env.get_matrix(self.baseline_env.task_dic, 'time_start'))
        perf_metrics['waiting_time'] = np.mean(self.baseline_env.get_matrix(self.baseline_env.agent_dic, 'sum_waiting_time'))
        perf_metrics['travel_dist'] = np.sum(self.baseline_env.get_matrix(self.baseline_env.agent_dic, 'travel_dist'))
        perf_metrics['efficiency'] = np.mean(self.baseline_env.get_matrix(self.baseline_env.task_dic, 'sum_waiting_time'))
        return perf_metrics

    def baseline_test(self):
        self.baseline_env.plot_figure = False
        perf_metrics = {}
        while not self.baseline_env.finished and self.baseline_env.current_time < MAX_TIME:
            with torch.no_grad():
                decision_agents, current_time = self.baseline_env.next_decision()
                groups = self.baseline_env.get_unique_group(decision_agents)
                self.baseline_env.current_time = current_time
                self.baseline_env.task_update()
                self.baseline_env.agent_update()
                for group in groups:
                    while len(group) > 0:
                        leader_id = np.random.choice(group)
                        agent = self.baseline_env.agent_dic[leader_id]
                        if not agent['returned']:
                            mask = self.baseline_env.get_unfinished_task_mask()
                            if np.sum(mask) == self.baseline_env.tasks_num:
                                mask = np.insert(mask, 0, False)
                            else:
                                mask = np.insert(mask, 0, True)
                            total_agents = torch.FloatTensor(self.baseline_env.get_current_agent_status(agent)).unsqueeze(0).to(self.device)
                            # total_agents = self.zero_padding(total_agents, AGENTS_RANGE[1])
                            task_info = torch.FloatTensor(self.baseline_env.get_current_task_status(agent)).unsqueeze(0).to(self.device)
                            # task_info = self.zero_padding(task_info, TASKS_RANGE[1] + 1)
                            mask = torch.tensor(mask).unsqueeze(0).to(self.device)
                            # mask = self.true_padding(mask, TASKS_RANGE[1] + 1)
                            agent_id = torch.tensor([[[agent['ID']]]]).to(self.device)
                            logp_list = self.local_net(task_info, total_agents, mask)
                            action = torch.argmax(logp_list, 1)
                            group, r = self.baseline_env.step(group, leader_id, action.item(), 0)
                            self.baseline_env.task_update()
                            self.baseline_env.agent_update()
                self.baseline_env.finished = self.baseline_env.check_finished()

        reward, finished_tasks = self.baseline_env.get_episode_reward(MAX_TIME)
        return reward

    def work(self, episode_number):
        """
        Interacts with the environment. The agent gets either gradients or experience buffer
        """
        self.episode_number = episode_number
        self.perf_metrics = self.run_episode(episode_number)

    def generate_route(self):
        route = self.baseline_env.get_matrix(self.baseline_env.agent_dic, 'route')
        for i in range(len(route)):
            route[i] = [iterator + 1 for iterator in route[i]]
        route = dict(enumerate(route))
        import yaml
        with open(f'route_ros_large.yaml', 'w') as f:
            yaml.dump(route, f, sort_keys=False)

    @staticmethod
    def zero_padding(a, max_len=AGENTS_RANGE[1]):
        # torch zero padding
        return F.pad(a, (0, 0, 0, max_len - a.shape[1]), 'constant', -1)

    @staticmethod
    def true_padding(a, max_len=TASKS_RANGE[1]):
        # torch true padding
        return F.pad(a, (0, max_len - a.shape[1]), 'constant', True)



if __name__ == '__main__':
    device = torch.device('cpu')
    # checkpoint = torch.load(model_path + '/checkpoint.pth')
    localNetwork = AttentionNet(AGENT_INPUT_DIM, TASK_INPUT_DIM, EMBEDDING_DIM).to(device)
    # localNetwork.load_state_dict(checkpoint['model'])
    for i in range(10):
        worker = Worker(1, localNetwork, localNetwork, 0, device=device, seed=i, save_image=False)
        worker.run_episode(i)
        print(i)
