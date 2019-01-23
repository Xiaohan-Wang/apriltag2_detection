import plotly.plotly as py
import plotly.graph_objs as go
import os
from ruamel import yaml
import numpy as np
import math


option = 3 # select from [1,2,3]
option_str = {1:'cpu_load (%)', 2:'virt_mem (MB)', 3:'real_mem (MB)'}  

groundtruth = {'x (cm)':[], 'y (cm)':[], 'oz (degree)':[]}
mean_value = [] # mean of cpu load / virtual memory / real memory 
variance = [] #variance
text = []

path_name = os.getcwd()
parent_path = os.path.dirname(path_name)
summary_path = os.path.join(parent_path, "test_result", "summary")

for filename in os.listdir(summary_path):
    # obtain groundtruth from filenames
    groundtruth['x (cm)'].append(float(filename.split('_')[2])) 
    groundtruth['y (cm)'].append(float(filename.split('_')[3]))
    groundtruth['oz (degree)'].append(float(filename.split('_')[4]))

    with open(summary_path + '/' + filename, 'r') as f:
        summary = yaml.load(f.read())
        data_mean = summary['cpu/ram'][option_str[option]]['mean']
        data_variance = np.var(summary['cpu/ram']['cpu_ram_info'][option_str[option]]['mean'])
        mean_value.append(data_mean)
        variance.append(data_variance)
        text.append('mean: ' + str(data_mean) + '<br>variance: ' + str(data_variance))


size = []
for i in range(len(variance)):
    size.append(math.log(variance[i], 10))
size=(np.array(size)-min(size))/(max(size)-min(size))*20+10

data = [
    {
        'x': groundtruth['x (cm)'],
        'y': groundtruth['y (cm)'],
        'text': text,
        'mode': 'markers',
        'marker': {
            'color': mean_value,  #mean value
            'size': size, 
            'showscale': True
        }
    }
]

#add title and axis name
layout = go.Layout(
    title = option_str[option] + ' (yaw= '+ str(groundtruth['oz (degree)'][0]) + ')',
    titlefont = {
            'size':30
    },
    xaxis = {
        'title':'x(cm)',
        'titlefont':{
            'size':20
        }
    },
    yaxis = {
        'title':'y(cm)',
        'titlefont':{
            'size':20
        }
    }
)

fig = go.Figure(data=data, layout=layout)

file_name = option_str[option] + ' (yaw= '+ str(groundtruth['oz (degree)'][0]) + ')'
py.plot(fig, filename=file_name)

