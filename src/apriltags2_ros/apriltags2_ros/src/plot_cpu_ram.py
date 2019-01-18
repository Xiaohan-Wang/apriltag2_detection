import plotly.plotly as py
from plotly import tools
import plotly.graph_objs as go
import os
from ruamel import yaml
import numpy as np
import copy

option = 1 # select from [1,2,3]
option_str = {1:'cpu_load (%)', 2:'virt_mem (MB)', 3:'real_mem (MB)'} 

path_name = os.getcwd()
parent_path = os.path.dirname(path_name)
summary_path = os.path.join(parent_path, "test_result", "summary")

# template of the annotation to show the position of max 
tem_annotation = {
    "font":{"size":15},
    "showarrow":False,
    "text":None,    # str(max_position*100)+'%',
    "x":None,
    "y":None
}

# a box plot for each x
for x in range(10,31,8):
    annotation = []
    fig = tools.make_subplots(rows=1, cols=2, subplot_titles=('max usage','mean usage'))

    for filename in os.listdir(summary_path):
        groundtruth_x = float(filename.split('_')[2])
        if groundtruth_x != x:
            continue;
        groundtruth_y = float(filename.split('_')[3])
        groundtruth_yaw = float(filename.split('_')[4])  

        with open(summary_path + '/' + filename, 'r') as f:
            summary = yaml.load(f.read())   
            data_mean = summary['cpu/ram']['cpu_ram_info'][option_str[option]]['mean']
            data_max = summary['cpu/ram']['cpu_ram_info'][option_str[option]]['max']
            total_num = summary['cpu/ram']['total number of cpu/ram messages']
            max_position =  round(float(summary['cpu/ram'][option_str[option]]['index of max']) / total_num + 0.05, 1)                
            
            temp = copy.copy(tem_annotation) 
            temp['text'] = str(max_position*100)+'%'
            temp['x'] = groundtruth_y 
            temp['y'] = max(data_max) + 5 
            annotation.append(temp)

            trace_mean = go.Box(
                            name = str(groundtruth_y), 
                            x = [groundtruth_y]*total_num,
            		        y = data_mean
            	        )
            trace_max =go.Box(
                            name = str(groundtruth_y), 
                            x = [groundtruth_y]*total_num,
            		        y = data_max
            	        )

            fig.append_trace(trace_max,1,1)
            fig.append_trace(trace_mean,1,2)

    #add title and axis name
    fig['layout']['xaxis1'].update(title='y(cm)', titlefont={'size':20})
    fig['layout']['yaxis1'].update(title='max ' + option_str[option], titlefont={'size':20}, autorange = True)
    fig['layout']['xaxis2'].update(title='y(cm)', titlefont={'size':20})
    fig['layout']['yaxis2'].update(title='mean ' + option_str[option], titlefont={'size':20}, autorange = True)
    fig['layout'].update(
        title = option_str[option] + ' (x= ' + str(x) + 'cm, yaw= '+ str(groundtruth_yaw) + ')', 
        titlefont={'size':30}
    )

    
    file_name = option_str[option] + " - x:" + str(10)
    py.plot(fig, filename=option_str[option] + ' (x= ' + str(x) + 'cm, yaw= '+ str(groundtruth_yaw) + ')')


