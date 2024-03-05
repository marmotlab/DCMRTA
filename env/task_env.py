import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.animation import FuncAnimation
from matplotlib.offsetbox import OffsetImage, AnnotationBbox


class TaskEnv:
    def __init__(self, agents_range=(10, 10), tasks_range=(10, 10), traits_dim=1, max_coalition_size=3, max_duration=5,
                 seed=None, plot_figure=False):
        """
        :param traits_dim: number of capabilities in this problem, e.g. 3 traits
        :param seed: seed to generate pseudo random problem instance
        """
        self.rng = None
        self.agents_range = agents_range
        self.tasks_range = tasks_range
        self.max_coalition_size = max_coalition_size
        self.max_duration = max_duration
        self.plot_figure = plot_figure
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.traits_dim = traits_dim
        self.task_dic, self.agent_dic, self.depot = self.generate_env()
        self.tasks_num = len(self.task_dic)
        self.agents_num = len(self.agent_dic)
        self.coalition_matrix = np.zeros((self.agents_num, self.tasks_num))
        self.current_time = 0
        self.dt = 0.1
        self.max_waiting_time = 10
        self.finished = False
        self.force_wait = True
        self.reactive_planning = False
        self.visible_length = 0

    def random_int(self, low, high, size=None):
        if self.rng is not None:
            integer = self.rng.integers(low, high, size)
        else:
            integer = np.random.randint(low, high, size)
        return integer

    def random_value(self, row, col):
        if self.rng is not None:
            value = self.rng.random((row, col))
        else:
            value = np.random.rand(row, col)
        return value

    def random_choice(self, a, size=None, replace=True):
        if self.rng is not None:
            choice = self.rng.choice(a, size, replace)
        else:
            choice = np.random.choice(a, size, replace)
        return choice

    def generate_env(self):
        if type(self.tasks_range) is tuple:
            tasks_num = self.random_int(self.tasks_range[0], self.tasks_range[1] + 1)
        else:
            tasks_num = self.tasks_range
        if type(self.agents_range) is tuple:
            agents_num = self.random_int(self.agents_range[0], self.agents_range[1] + 1)
        else:
            agents_num = self.agents_range
        agents_ini = np.ones((agents_num, self.traits_dim))
        depot = self.random_value(1, 2)
        cost_ini = self.random_value(agents_num, 1)
        tasks_loc = self.random_value(tasks_num, 2)
        tasks_time = np.ones((tasks_num, 1)) * self.max_duration
        tasks_ini = self.random_int(1, self.max_coalition_size + 1, tasks_num).reshape(-1, self.traits_dim)

        task_dic = dict()
        agent_dic = dict()
        for i in range(tasks_num):
            task_dic[i] = {'ID': i,
                           'requirements': tasks_ini[i, :],  # requirements of the task
                           'members': [],  # members of the task
                           'cost': [],  # cost of each agent
                           'location': tasks_loc[i, :],  # location of the task
                           'feasible_assignment': False,  # whether the task assignment is feasible
                           'finished': False,
                           'time_start': 0,
                           'time_finish': 0,
                           'status': tasks_ini[i, :],
                           'time': float(tasks_time[i, :]),
                           'sum_waiting_time': 0,
                           'efficiency': 0,
                           'abandoned_agent': []}
        for i in range(agents_num):
            agent_dic[i] = {'ID': i,
                            'abilities': agents_ini[i, :],
                            'location': depot[0, :],
                            'next_location': depot[0, :],
                            'route': [],
                            'arrival_time': [],
                            'cost': cost_ini[i],
                            'travel_time': 0,
                            'velocity': 0.2,
                            'next_decision': 0,
                            'depot': depot[0, :],
                            'travel_dist': 0,
                            'sum_waiting_time': 0,
                            'current_action_index': 0,
                            'working_condition': 0,
                            'trajectory': [],
                            'angle': 0,
                            'returned': False,
                            'assigned': False,
                            'pre_set_route': None}
        depot = {'location': depot[0, :],
                 'members': [],
                 'ID': -1}
        return task_dic, agent_dic, depot

    def reset(self, test_env=None, seed=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        else:
            self.rng = None
        if test_env is not None:
            self.task_dic, self.agent_dic, self.depot = test_env
        self.tasks_num = len(self.task_dic)
        self.agents_num = len(self.agent_dic)
        self.coalition_matrix = np.zeros((self.agents_num, self.tasks_num))
        self.current_time = 0
        self.finished = False

    def clear_decisions(self):
        for task in self.task_dic.values():
            task.update(members=[], cost=[], finished=False, status=task['requirements'],feasible_assignment=False,
                        time_start=0, time_finish=0, sum_waiting_time=0, efficiency=0, abandoned_agent=[])
        for agent in self.agent_dic.values():
            agent.update(route=[], location=self.depot['location'], next_location=self.depot['location'],
                         next_decision=0, travel_time=0, travel_dist=0, arrival_time=[], assigned=False,
                         sum_waiting_time=0, working_condition=0, current_action_index=0,
                         trajectory=[], angle=0, returned=False, pre_set_route=None, depot=self.depot['location'])
        self.depot.update(members=[], ID=-1)
        self.current_time = 0
        self.finished = False

    @staticmethod
    def find_by_key(data, target):
        for key, value in data.items():
            if isinstance(value, dict):
                yield from TaskEnv.find_by_key(value, target)
            elif key == target:
                yield value

    @staticmethod
    def get_matrix(dictionary, key):
        """
        :param key: the key to index
        :param dictionary: the dictionary for key to index
        """
        key_matrix = []
        for value in dictionary.values():
            key_matrix.append(value[key])
        return key_matrix

    @staticmethod
    def calculate_eulidean_distance(agent, task):
        return np.linalg.norm(agent['location'] - task['location'])

    def get_current_agent_status(self, agent):
        status = []
        for a in self.agent_dic.values():
            if len(a['route']) > 0 and a['route'][-1] in self.task_dic.keys():
                travel_time = np.clip(self.get_arrival_time(a['ID'], a['route'][-1]) - self.current_time, a_min=0, a_max=None)
                current_waiting_time = np.clip(self.current_time - self.get_arrival_time(a['ID'], a['route'][-1]), a_min=0, a_max=None) if self.current_time <= self.task_dic[a['route'][-1]]['time_start'] else 0
                remaining_working_time = np.clip(self.task_dic[a['route'][-1]]['time_start'] + self.task_dic[a['route'][-1]]['time'] - self.current_time, a_min=0, a_max=None) if self.current_time >= self.task_dic[a['route'][-1]]['time_start'] else 0
            else:
                travel_time = 0
                current_waiting_time = 0
                remaining_working_time = 0
            temp_status = np.hstack([travel_time, remaining_working_time, current_waiting_time,
                                     agent['location'] - a['location'], a['assigned']])
            status.append(temp_status)
        current_agents = np.vstack(status)
        return current_agents

    def get_current_task_status(self, agent):
        status = []
        for t in self.task_dic.values():
            temp_status = np.hstack([t['status'], t['requirements'], t['time'],
                                     t['location'] - agent['location']])
            status.append(temp_status)
        status = [np.hstack([0, 0, 0, self.depot['location'] - agent['location']])] + status
        current_tasks = np.vstack(status)
        return current_tasks

    def get_unfinished_task_mask(self):
        mask = np.logical_not(self.get_unfinished_tasks())
        return mask

    def get_unfinished_tasks(self):
        unfinished_tasks = []
        for task in self.task_dic.values():
            unfinished_tasks.append(task['feasible_assignment'] is False and np.any(task['status'] > 0))
        return unfinished_tasks

    def get_arrival_time(self, agent_id, task_id):
        arrival_time = self.agent_dic[agent_id]['arrival_time']
        arrival_for_task = np.where(np.array(self.agent_dic[agent_id]['route']) == task_id)[0][-1]
        return float(arrival_time[arrival_for_task])

    def agent_update(self):
        for agent in self.agent_dic.values():
            if len(agent['arrival_time']) > 0:
                time_difference = agent['arrival_time'][-1] - self.current_time
                agent['working_condition'] = time_difference
                if agent['route'][-1] == -1:
                    if self.reactive_planning:
                        if np.all(self.get_matrix(self.task_dic, 'feasible_assignment')[:self.visible_length]):
                            agent['next_decision'] = np.nan
                        else:
                            if agent['pre_set_route'] is not None and not agent['pre_set_route']:
                                agent['next_decision'] = np.nan
                            else:
                                next_action = agent['pre_set_route'][0]
                                next_decision_time = (next_action - 1)//20 * 10
                                agent['next_decision'] = np.max([self.get_arrival_time(agent['ID'], -1), next_decision_time, self.current_time])
                                if agent['ID'] in self.depot['members']:
                                    self.depot['members'].remove(agent['ID'])
                    else:
                        agent['next_decision'] = np.nan
                else:
                    current_task = self.task_dic[agent['route'][-1]]
                    if current_task['feasible_assignment']:
                        if agent['ID'] in current_task['members']:
                            agent['next_decision'] = float(current_task['time_finish'])
                            if self.current_time >= float(current_task['time_start']):
                                agent['assigned'] = True
                        else:
                            agent['next_decision'] = self.get_arrival_time(agent['ID'], current_task['ID']) + self.max_waiting_time
                            agent['assigned'] = False
                    else:
                        agent['next_decision'] = self.get_arrival_time(agent['ID'], current_task['ID']) + \
                                                 self.max_waiting_time
                        agent['assigned'] = False

            else:
                agent['working_condition'] = 0.

    def task_update(self):
        f = []
        # check each task status and whether it is finished
        for task in self.task_dic.values():
            if not task['feasible_assignment']:
                abilities = len(task['members'])
                arrival = np.array([self.get_arrival_time(member, task['ID']) for member in task['members']])
                task['status'] = task['requirements'] - abilities  # update task status
                # Agents will wait for the other agents to arrive
                if task['status'] <= 0:
                    if np.max(arrival) - np.min(arrival) <= self.max_waiting_time:
                        task['time_start'] = float(np.max(arrival, keepdims=True))
                        task['time_finish'] = float(np.max(arrival, keepdims=True) + task['time'])
                        task['feasible_assignment'] = True
                        f.append(task['ID'])
                    else:
                        task['feasible_assignment'] = False
                        infeasible_members = arrival <= np.max(arrival, keepdims=True) - self.max_waiting_time
                        for member in np.array(task['members'])[infeasible_members]:
                            task['members'].remove(member)
                            task['abandoned_agent'].append(member)
                else:
                    task['feasible_assignment'] = False
                    for member in task['members']:
                        if self.current_time - self.get_arrival_time(member, task['ID']) >= self.max_waiting_time:
                            task['members'].remove(member)
                            task['abandoned_agent'].append(member)
            else:
                if self.current_time >= task['time_finish']:
                    task['finished'] = True

        # check depot status
        depot = self.depot
        for member in depot['members']:
            if self.current_time >= self.get_arrival_time(member, -1) and np.all(self.get_matrix(self.task_dic, 'feasible_assignment')):
                self.agent_dic[member]['returned'] = True
        return f

    def next_decision(self):
        decision_time = np.array(self.get_matrix(self.agent_dic, 'next_decision'))
        if np.all(np.isnan(decision_time)):
            return [], max(map(lambda x: max(x) if x else 0, self.get_matrix(self.agent_dic, 'arrival_time')))
        next_decision = np.nanmin(decision_time)
        agents = np.where(decision_time == next_decision)[0]
        return agents, next_decision

    def get_unique_group(self, agents):
        location = np.array(self.get_matrix(self.agent_dic, 'location'))[agents]
        unique_location = np.unique(location, axis=0)
        # agent group in each unique location
        unique_group = []
        for loc in unique_location:
            unique_group.append(agents[np.where(np.all(location == loc, axis=1))[0].tolist()].tolist())
        return unique_group

    def agent_step(self, agent_id, task_id):
        """
        :param agent_id: the id of agent
        :param task_id: the id of task
        :return: end_episode, finished_tasks
        """
        #  choose any task
        task_id = task_id - 1
        if task_id != -1:
            agent = self.agent_dic[agent_id]
            task = self.task_dic[task_id]
        else:
            agent = self.agent_dic[agent_id]
            task = self.depot
        agent['route'].append(task_id)
        travel_time = self.calculate_eulidean_distance(agent, task) / agent['velocity']
        agent['travel_time'] = travel_time
        agent['travel_dist'] += self.calculate_eulidean_distance(agent, task)
        agent['arrival_time'] += [self.current_time + travel_time]
        # calculate the angle from current location to next location
        agent['location'] = task['location']
        if agent_id not in task['members']:
            task['members'].append(agent_id)

        return - travel_time

    def step(self, group, leader_id, action, current_action_index=0):
        vacancy = self.task_dic[action-1]['status'] if action-1 in self.task_dic.keys() else len(group)
        group.remove(leader_id)
        available_agents = len(group)
        if vacancy > 1:
            followers = self.random_choice(group, np.minimum(vacancy - 1, available_agents), False).tolist()
            for follower in followers:
                group.remove(follower)
            members = [leader_id] + followers
        else:
            members = [leader_id]
        reward = 0
        for member in members:
            reward += self.agent_step(member, action)
            self.agent_dic[member]['current_action_index'] = current_action_index
        reward = reward / len(members)
        return group, reward

    def calculate_waiting_time(self):
        for agent in self.agent_dic.values():
            agent['sum_waiting_time'] = 0
        for task in self.task_dic.values():
            arrival = np.array([self.get_arrival_time(member, task['ID']) for member in task['members']])
            if len(arrival) != 0:
                if task['feasible_assignment']:
                    task['sum_waiting_time'] = np.sum(np.max(arrival) - arrival) \
                                               + len(task['abandoned_agent']) * self.max_waiting_time
                else:
                    task['sum_waiting_time'] = np.sum(self.current_time - arrival) \
                                               + len(task['abandoned_agent']) * self.max_waiting_time
            else:
                task['sum_waiting_time'] = len(task['abandoned_agent']) * self.max_waiting_time
            for member in task['members']:
                if task['feasible_assignment']:
                    self.agent_dic[member]['sum_waiting_time'] += np.max(arrival) - self.get_arrival_time(member, task['ID'])
                else:
                    self.agent_dic[member]['sum_waiting_time'] += self.current_time - self.get_arrival_time(member, task['ID']) if self.current_time - self.get_arrival_time(member, task['ID']) > 0 else 0
            for member in task['abandoned_agent']:
                self.agent_dic[member]['sum_waiting_time'] += self.max_waiting_time

    def check_finished(self):
        decision_agents, current_time = self.next_decision()
        if len(decision_agents) == 0:
            self.current_time = current_time
            finished = np.all(self.get_matrix(self.agent_dic, 'returned')) and np.all(self.get_matrix(self.task_dic, 'finished'))
        else:
            finished = False
        return finished

    def generate_traj(self):
        for agent in self.agent_dic.values():
            # save the location of the agent as trajectory
            time_step = 0
            for i in range(len(agent['route'])):
                previous_task = self.task_dic[agent['route'][i - 1]] if i > 0 and agent['route'][i - 1] != -1 else self.depot
                current_task = self.task_dic[agent['route'][i]] if agent['route'][i] != -1 else self.depot
                angle = np.arctan2(current_task['location'][1] - previous_task['location'][1],
                                   current_task['location'][0] - previous_task['location'][0])
                distance = self.calculate_eulidean_distance(previous_task, current_task)
                total_time = distance / agent['velocity']
                arrival_time_current = agent['arrival_time'][i]
                arrival_time_prev = agent['arrival_time'][i - 1] if previous_task['ID'] != -1 else 0
                if current_task['ID'] != -1 and agent['ID'] in current_task['members'] \
                        and current_task['feasible_assignment']:
                    if current_task['time_start'] - arrival_time_current <= self.max_waiting_time:
                        next_decision = current_task['time_finish']
                    else:
                        next_decision = arrival_time_current + self.max_waiting_time
                else:
                    next_decision = arrival_time_current + self.max_waiting_time
                if previous_task['ID'] == -1:
                    prev_decision = 0
                else:
                    if agent['ID'] in previous_task['members'] \
                            and previous_task['time_start'] - arrival_time_prev <= self.max_waiting_time \
                            and previous_task['feasible_assignment']:
                        prev_decision = previous_task['time_finish']
                    else:
                        prev_decision = arrival_time_prev + self.max_waiting_time
                while time_step < next_decision:
                    time_step += self.dt
                    if time_step < arrival_time_current:
                        fraction_of_time = (time_step - prev_decision) / total_time
                        x = previous_task['location'][0] + fraction_of_time * (
                                    current_task['location'][0] - previous_task['location'][0])
                        y = previous_task['location'][1] + fraction_of_time * (
                                    current_task['location'][1] - previous_task['location'][1])
                        agent['trajectory'].append(np.hstack([x, y, angle]))
                    else:
                        agent['trajectory'].append(np.array([current_task['location'][0], current_task['location'][1], angle]))
            while time_step < self.current_time:
                time_step += self.dt
                agent['trajectory'].append(np.array([self.depot['location'][0], self.depot['location'][1], angle]))

    def get_episode_reward(self, max_time=100):
        self.calculate_waiting_time()
        end = self.check_finished()
        finished_tasks = self.get_matrix(self.task_dic, 'finished')
        reward = - self.current_time
        return reward, finished_tasks

    def stack_trajectory(self):
        for agent in self.agent_dic.values():
            agent['trajectory'] = np.vstack(agent['trajectory'])

    def plot_animation(self, path, n):
        self.generate_traj()
        plot_robot_icon = False
        if plot_robot_icon:
            drone = plt.imread('env/drone.png')
            drone_oi = OffsetImage(drone, zoom=0.05)

        # Set up the plot
        self.stack_trajectory()
        finished_tasks = self.get_matrix(self.task_dic, 'finished')
        finished_rate = np.sum(finished_tasks) / len(finished_tasks)
        gif_len = int(self.current_time/self.dt)
        fig, ax = plt.subplots(dpi=100)
        ax.set_xlim(-0.5, 10.5)
        ax.set_ylim(-0.5, 10.5)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_aspect('equal')
        plt.subplots_adjust(left=0, right=0.85, top=0.87, bottom=0.02)
        lines = [ax.plot([], [], color='teal', zorder=0)[0] for _ in self.agent_dic.values()]
        ax.set_title(f'Agents finish {finished_rate * 100}% tasks within {self.current_time:.2f}min.'
                     f'\nCurrent time is {0:.2f}min')
        green_patch = patches.Patch(color='g', label='Finished task')
        blue_patch = patches.Patch(color='b', label='Unfinished task')
        red_patch = patches.Patch(color='r', label='Single agent')
        yellow_patch = patches.Patch(color='y', label='Two agents')
        cyan_patch = patches.Patch(color='c', label='Three agents')
        magenta_patch = patches.Patch(color='m', label='>= Four agents')
        if plot_robot_icon:
            ax.legend(handles=[green_patch, blue_patch], bbox_to_anchor=(0.99, 0.7))
        else:
            ax.legend(handles=[green_patch, blue_patch, red_patch, yellow_patch, cyan_patch, magenta_patch],
                      bbox_to_anchor=(0.99, 0.7))
        task_squares = [ax.add_patch(patches.RegularPolygon(xy=(task['location'][0] * 10,
                                                             task['location'][1] * 10),
                                                            numVertices=int(task['requirements'].sum()) + 3,
                                                            radius=0.3, color='b')) for task in self.task_dic.values()]
        ax.add_patch(patches.Circle((self.depot['location'][0] * 10, self.depot['location'][1] * 10),
                                    0.2,
                                    color='r'))
        if plot_robot_icon:
            agent_triangles = []
            for a in self.agent_dic.values():
                agent_triangles.append(ax.add_artist(AnnotationBbox(drone_oi, (self.depot['location'][0] * 10,
                                                     self.depot['location'][1] * 10),
                                       frameon=False)))
        else:
            agent_triangles = [ax.add_patch(patches.RegularPolygon(xy=(self.depot['location'][0] * 10,
                                                                       self.depot['location'][1] * 10), numVertices=3,
                                                                   radius=0.2, color='r'))
                               for _ in self.agent_dic.values()]

        # Define the update function for the animation
        def update(frame):
            ax.set_title(f'Agents finish {finished_rate * 100}% tasks within {self.current_time:.2f}min.'
                         f'\nCurrent time is {frame * self.dt:.2f}min')
            pos = np.round([agent['trajectory'][frame, 0:2] for agent in self.agent_dic.values()], 4)
            unq, count = np.unique(pos, axis=0, return_counts=True)
            for agent in self.agent_dic.values():
                repeats = int(count[np.argwhere(np.all(unq == np.round(agent['trajectory'][frame, 0:2], 4), axis=1))])
                agent_triangles[agent['ID']].xy = tuple(agent['trajectory'][frame, 0:2] * 10)
                if plot_robot_icon:
                    agent_triangles[agent['ID']].xyann = tuple(agent['trajectory'][frame, 0:2] * 10)
                    agent_triangles[agent['ID']].xybox = tuple(agent['trajectory'][frame, 0:2] * 10)
                else:
                    agent_triangles[agent['ID']].set_color('m' if repeats >= 4 else 'c' if repeats == 3
                                                           else 'y' if repeats == 2 else 'r')
                agent_triangles[agent['ID']].orientation = agent['trajectory'][frame, 2] - np.pi / 2
                # Add the current frame's data point to the plot for each trajectory
                if frame > 40:
                    lines[agent['ID']].set_data(agent['trajectory'][frame - 40:frame + 1, 0] * 10,
                                                agent['trajectory'][frame - 40:frame + 1, 1] * 10)
                else:
                    lines[agent['ID']].set_data(agent['trajectory'][:frame + 1, 0] * 10,
                                                agent['trajectory'][:frame + 1, 1] * 10)

            for task in self.task_dic.values():
                if self.reactive_planning:
                    if task['ID'] > np.clip(frame * self.dt//10 * 20 + 20, 20, 100):
                        task_squares[task['ID']].set_color('w')
                        task_squares[task['ID']].set_zorder(0)
                    else:
                        task_squares[task['ID']].set_color('b')
                        task_squares[task['ID']].set_zorder(1)
                if frame * self.dt >= task['time_finish'] > 0:
                    task_squares[task['ID']].set_color('g')
            return lines

        # Set up the animation
        ani = FuncAnimation(fig, update, frames=gif_len, interval=100, blit=True)
        ani.save(f'{path}/episode_{n}_{self.current_time:.1f}.gif')

    def get_grouped_tasks(self):
        grouped_tasks = dict()
        groups = list(set(np.array(self.get_matrix(self.task_dic, 'requirements')).squeeze(1).tolist()))
        for task_requirement in groups:
            grouped_tasks[task_requirement] = dict()
        index = np.zeros_like(groups)
        for i, task in self.task_dic.items():
            requirement = int(task['requirements'])
            ind = index[groups.index(requirement)]
            grouped_tasks[requirement].update({ind: task})
            index[groups.index(requirement)] += 1
        grouped_tasks = {key: value for key, value in grouped_tasks.items() if len(value) > 0}
        agent_v = {}
        if np.sum(list(grouped_tasks.keys())) > self.agents_num:
            agent_num = self.agents_num * 2
        else:
            agent_num = self.agents_num
        for keys, values in grouped_tasks.items():
            agent_v[keys] = (len(values) / self.tasks_num + np.sum(self.get_matrix(values, 'time')) / np.sum(self.get_matrix(self.task_dic, 'time'))) * keys
        agent_v_ = np.array(list(agent_v.values()))
        agent_v_ = agent_v_ / np.sum(agent_v_) * agent_num
        agent_v_ = np.clip(agent_v_, list(agent_v.keys()), None)
        agent_v_r = agent_v_ // np.array(list(agent_v.keys()))
        remainder = agent_v_ % np.array(list(agent_v.keys())) / np.array(list(agent_v.keys()))
        rest_ = agent_num - np.dot(agent_v_r, np.array(list(agent_v.keys())))
        sort_ = np.argsort(remainder)[::-1]
        while rest_ != 0 and np.any(np.array(list(agent_v.keys())) // rest_ == 0):
            for idx in sort_:
                add_ = np.clip(rest_ // list(agent_v.keys())[idx], 0, 1)
                agent_v_r[idx] += add_
                rest_ -= add_ * list(agent_v.keys())[idx]
        grouped_agents = {}
        for i, (keys, values) in enumerate(grouped_tasks.items()):
            if agent_v_r[i] == 0 and len(grouped_tasks[keys]) != 0:
                grouped_agents[keys] = 1
            else:
                grouped_agents[keys] = int(agent_v_r[i])
        return grouped_tasks, grouped_agents

    def execute_by_route(self, path='./', method=0, plot_figure=False):
        self.plot_figure = plot_figure
        self.max_waiting_time = 100
        while not self.finished and self.current_time < 200:
            if self.reactive_planning:
                self.visible_length = int(np.clip(self.current_time//10 * 20 + 20, 20, 100))
            decision_agents, current_time = self.next_decision()
            self.current_time = current_time
            self.task_update()
            self.agent_update()
            for agent in decision_agents:
                if self.agent_dic[agent]['pre_set_route'] is None or not self.agent_dic[agent]['pre_set_route']:
                    self.agent_step(agent, 0)
                    self.task_update()
                    self.agent_update()
                    continue
                if self.reactive_planning:
                    if self.agent_dic[agent]['pre_set_route']:
                        if self.agent_dic[agent]['pre_set_route'][0] > self.visible_length:
                            self.agent_step(agent, 0)
                            self.task_update()
                            self.agent_update()
                            continue
                self.agent_step(agent, self.agent_dic[agent]['pre_set_route'].pop(0))
                self.task_update()
                self.agent_update()
            self.finished = self.check_finished()
        if self.plot_figure:
            self.plot_animation(path, method)
        print(self.current_time)
        # self.process_map(path)
        return self.current_time

    def pre_set_route(self, routes, agent_id):
        if not self.agent_dic[agent_id]['pre_set_route']:
            self.agent_dic[agent_id]['pre_set_route'] = routes
        else:
            self.agent_dic[agent_id]['pre_set_route'] += routes

    def process_map(self, path):
        import pandas as pd
        grouped_tasks = dict()
        groups = list(set(np.array(self.get_matrix(self.task_dic, 'requirements')).squeeze(1).tolist()))
        for task_requirement in groups:
            grouped_tasks[task_requirement] = dict()
        index = np.zeros_like(groups)
        for i, task in self.task_dic.items():
            requirement = int(task['requirements'])
            ind = index[groups.index(requirement)]
            grouped_tasks[requirement].update({ind: task})
            index[groups.index(requirement)] += 1
        grouped_tasks = {key: value for key, value in grouped_tasks.items() if len(value) > 0}
        time_finished = [self.get_matrix(dic, 'time_finish') for dic in grouped_tasks.values()]
        t = 0
        time_tick_stamp = dict()
        while t <= self.current_time:
            time_tick_stamp[t] = [np.sum(np.array(ratio) < t)/len(ratio) for ratio in time_finished]
            t += 0.1
            t = np.round(t, 1)
        pd = pd.DataFrame(time_tick_stamp)
        pd.to_csv(f'{path}time_RL.csv')

