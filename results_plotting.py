import glob
import os
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from scipy.stats import ttest_rel
from natsort import natsorted

# plot all csv results in a folder and plot all same label in one figure
folder = 'testSet_20A_50T_CONDET'
if not os.path.exists(f'{folder}/metrics'):
    os.mkdir(f'{folder}/metrics')
dfs = []
files = []
labels = ['Success Rate', 'Makespan', 'Time Cost', 'Average Waiting Time', 'Sum Traveling Distance', 'Efficiency']
labels_in_csv = ['success_rate', 'makespan', 'time_cost', 'waiting_time', 'travel_dist', 'efficiency']
methods = natsorted(glob.glob(f'{folder}/*.csv'), key=lambda y: y.lower())
for file in methods:
    if file.endswith('.csv'):
        files.append(file.split('/')[1].replace('.csv', ''))
        dfs.append(pd.read_csv(file))

p_metrics = pd.DataFrame(columns=['Method'] + files)
for m, label_csv in enumerate(labels_in_csv):
    for i, df_i in enumerate(dfs):
        p = dict()
        for j, df_j in enumerate(dfs):
            if df_i is not df_j:
                result = ttest_rel(df_i[label_csv].values, df_j[label_csv].values)
                # result = ttest_rel(df_i[label_csv][~df_i[label_csv].isnan()].values, df_j[label_csv][~df_i[label_csv].isna()].values)
                p[files[j]] = np.format_float_scientific(result.statistic, 2) + ', ' + np.format_float_scientific(result.pvalue, 2)
            else:
                p[files[j]] = '0, 0'
        p['Method'] = files[i] + ' ' + labels[m]
        p = pd.DataFrame(p, index=[files[i]])
        p_metrics = pd.concat([p_metrics, p])
p_metrics.to_csv(f'{folder}/metrics/p_metrics.csv', index=False)


metrics_csv = pd.DataFrame(columns=['Method'] + labels)

for i, df in enumerate(dfs):
    metrics = dict()
    for j, label in enumerate(labels_in_csv):
        if label == 'success_rate':
            metrics[labels[j]] = (np.sum(df[label])/len(df[label])).round(3).astype('str') + ' (+- ' + np.nanstd(df[label]).round(3).astype('str') + ')'
        else:
            metrics[labels[j]] = np.nanmean(df[label]).round(3).astype('str') + ' (+- ' + np.nanstd(df[label]).round(3).astype('str') + ')'
    metrics['Method'] = files[i]
    metrics = pd.DataFrame(metrics, index=[files[i]])
    metrics_csv = pd.concat([metrics_csv, metrics])
metrics_csv.to_csv(f'{folder}/metrics/metrics.csv', index=False)

for metrics, label in enumerate(['success_rate', 'makespan', 'time_cost', 'waiting_time', 'travel_dist', 'efficiency']):
    plt.figure(dpi=300)
    for id, df in enumerate(dfs):
        plt.plot(df[label], label=files[id])
    plt.legend()
    plt.title(labels[metrics])
    plt.savefig(f'{folder}/metrics/{labels[metrics]}.png')
    plt.close()

# plot average results of all csv files in a folder and error bar
for metrics, label in enumerate(['success_rate', 'makespan', 'time_cost', 'waiting_time', 'travel_dist', 'efficiency']):
    plt.figure(dpi=300)
    for idx, df in enumerate(dfs):
        # plot average and error bar
        mean = np.nanmean(df[label])
        std = np.nanstd(df[label])
        min_ = np.min(df[label])
        max_ = np.max(df[label])
        plt.errorbar(idx, mean, std, fmt='b', lw=3, alpha=0.5)
        plt.errorbar(idx, mean, np.array([[np.round(mean - min_, 4)], [np.round(max_ - mean, 4)]]), fmt='.', lw=1, label=files[idx])
        # plt.errorbar(idx, mean, np.array([[np.round(mean - min_, 4)], [np.round(max_ - mean, 4)]]), fmt='.', lw=1,
        #             label=files[idx]+f' v={np.format_float_scientific(p[idx].statistic[metrics], 4)}  p={np.format_float_scientific(p[idx].pvalue[metrics], 4)}' if p[idx] is not None else files[idx])
    plt.legend(fontsize="7")
    plt.xticks([])
    plt.title(labels[metrics])
    plt.savefig(f'{folder}/metrics/{labels[metrics]} Average.png')
    plt.close()
