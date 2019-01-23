import plotly.plotly as py
from plotly import tools
import plotly.graph_objs as go
import os
from ruamel import yaml
import numpy as np
import copy

plot_x = 30
option = 2 # select from [1,2]
option_name = {1:'decimate rate - processing time', 2:'x (cm) - processing time'}
subprocess_name = {
    0: 'init',
    1: 'decimate',
    2: 'blur/sharp',
    3: 'threshold',
    4: 'unionfind',
    5: 'make clusters',
    6: 'fit quads to clusters',
    7: 'quads',
    8: 'decode+refinement',
    9: 'reconcile',
    10: 'debug output',
    11: 'cleanup',
    12: 'relative pose estimation'
}  

#summary file path
summary_path = '/home/xiaohan/duckietown/apriltag2_detection/src/apriltags2_ros/apriltags2_ros/test_result/summary' 

mean_time = []
max_time = []
for i in range(0,13):
    mean_time.append([])
    max_time.append([])
decimate_rate = []
x = []

fig = tools.make_subplots(rows=1, cols=2, subplot_titles=('average time usage','maximum time usage'))
for filename in os.listdir(summary_path):
    groundtruth_x = float(filename.split('_')[2])
    if(groundtruth_x != plot_x):
        continue;
    groundtruth_y = float(filename.split('_')[3])
    groundtruth_yaw = float(filename.split('_')[4])
    decimate = float(filename.split('_')[5])
    x.append(groundtruth_x) 
    decimate_rate.append(decimate)  

    with open(summary_path + '/' + filename, 'r') as f:
        summary = yaml.load(f.read()) 
        for i in range(3,13):
            mean_time[i].append(summary['time consumption (ms)'][subprocess_name[i]]['mean'])
            max_time[i].append(summary['time consumption (ms)'][subprocess_name[i]]['max'])

for i in range(0,13):
    if option == 1:
        x_data = decimate_rate
    else:
        x_data = x
    trace_mean = go.Bar(
                    x = x_data,
                    y = mean_time[i],
                    name = subprocess_name[i]
                )
    trace_max =go.Bar(
                    x = x_data,
                    y = max_time[i],
                    name = subprocess_name[i]
                )        
    fig.append_trace(trace_mean,1,1)
    fig.append_trace(trace_max,1,2)

#add title and axis name
if option == 1:
    fig['layout']['xaxis1'].update(title='decimate rate', titlefont={'size':20})
    fig['layout']['xaxis2'].update(title='decimate rate', titlefont={'size':20})
else:
    fig['layout']['xaxis1'].update(title='x (cm)', titlefont={'size':20})
    fig['layout']['xaxis2'].update(title='x (cm)', titlefont={'size':20})

fig['layout']['yaxis1'].update(title='average time usage (ms)', titlefont={'size':20}, autorange = True)
fig['layout']['yaxis2'].update(title='maximum time usage (ms)', titlefont={'size':20}, autorange = True)

fig['layout'].update(
    title = option_name[option] + '(x=' + str(groundtruth_x) + ' y=0 yaw=0)', 
    titlefont={'size':30},
    barmode='stack'
)

py.plot(fig, filename=option_name[option] + '(x=' + str(groundtruth_x) + ' y=0 yaw=0)')


