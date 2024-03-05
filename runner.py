import torch
import numpy as np
import ray
import os
from attention import AttentionNet
from worker import Worker
from parameters import *
from env.task_env import TaskEnv


class Runner(object):
    """Actor object to start running simulation on workers.
    Gradient computation is also executed on this object."""

    def __init__(self, metaAgentID):
        self.metaAgentID = metaAgentID
        self.device = torch.device('cuda') if USE_GPU else torch.device('cpu')
        self.localNetwork = AttentionNet(AGENT_INPUT_DIM, TASK_INPUT_DIM, EMBEDDING_DIM)
        self.localNetwork.to(self.device)
        self.localBaseline = AttentionNet(AGENT_INPUT_DIM, TASK_INPUT_DIM, EMBEDDING_DIM)
        self.localBaseline.to(self.device)

    def get_weights(self):
        return self.localNetwork.state_dict()

    def set_weights(self, weights):
        self.localNetwork.load_state_dict(weights)

    def set_baseline_weights(self, weights):
        self.localBaseline.load_state_dict(weights)

    def singleThreadedJob(self, episodeNumber, agents_num, tasks_num):
        save_img = False
        if SAVE_IMG:
            if episodeNumber % SAVE_IMG_GAP == 0:
                save_img = True
        worker = Worker(self.metaAgentID, self.localNetwork, self.localBaseline,
                        episodeNumber, self.device, save_img, agents_num, tasks_num)
        worker.work(episodeNumber)

        jobResults = worker.experience
        perf_metrics = worker.perf_metrics
        return jobResults, perf_metrics

    def testing(self, agents_range=AGENTS_RANGE, tasks_range=TASKS_RANGE, seed=None):
        worker = Worker(self.metaAgentID, self.localNetwork, self.localBaseline,
                        0, self.device, False, agents_num=agents_range, tasks_num=tasks_range, seed=seed)
        reward = worker.baseline_test()
        return reward

    def comparison(self, testing_ep, sample, sample_number, env_params):
        worker = Worker(self.metaAgentID, self.localNetwork, self.localBaseline,
                        0, self.device, False)
        env = TaskEnv(*env_params)
        reward = worker.run_test(testing_ep, env, sample, sample_number)
        return reward, self.metaAgentID

    def job(self, global_weights, baseline_weights, episodeNumber, agents_num, tasks_num):
        print("starting episode {} on metaAgent {}".format(episodeNumber, self.metaAgentID))
        # set the local weights to the global weight values from the master network
        self.set_weights(global_weights)
        self.set_baseline_weights(baseline_weights)

        jobResults, metrics = self.singleThreadedJob(episodeNumber, agents_num, tasks_num)

        info = {
            "id": self.metaAgentID,
            "episode_number": episodeNumber,
        }

        return jobResults, metrics, info


@ray.remote(num_cpus=1, num_gpus=NUM_GPU / NUM_META_AGENT)
class RLRunner(Runner):
    def __init__(self, metaAgentID):
        super().__init__(metaAgentID)


if __name__ == '__main__':
    ray.init()
    runner = RLRunner.remote(0)
    job_id = runner.singleThreadedJob.remote(1)
    out = ray.get(job_id)
    print(out[1])
