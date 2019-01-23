import plotly.plotly as py
from plotly import tools
import plotly.graph_objs as go
import os
from ruamel import yaml
import numpy as np
import copy

option = 3 # select from [1,2,3]
mismatch_option = {1:'x (cm)', 2:'y (cm)', 3:'oz (degree)'}  # test option;  oz represents yaw

#summary file path
summary_path = '/home/xiaohan/duckietown/apriltag2_detection/src/apriltags2_ros/apriltags2_ros/test_result/summary' 

decimate_list = [1, 2, 3, 4]
# 4*4 array. Each row corresponds to one position (x=15, 20, 25, 30) with different decimate
mismatch = [[0 for i in range(4)] for j in range(4)]  
variance = [[0 for i in range(4)] for j in range(4)]  

for filename in os.listdir(summary_path):
    groundtruth_x = float(filename.split('_')[2])
    # groundtruth_x is the distance between camera and apriltag, we need to add 10 cm to get distance between duckiebot and apriltag
    groundtruth_x += 10
    groundtruth_y = float(filename.split('_')[3])
    groundtruth_yaw = float(filename.split('_')[4])
    groundtruth = [groundtruth_x, groundtruth_y, groundtruth_yaw]
    decimate = int(float(filename.split('_')[5]))
    

    with open(summary_path + '/' + filename, 'r') as f:
        summary = yaml.load(f.read()) 
        test_mean = summary['apriltag_output'][mismatch_option[option]]['mean'] 
        test_groundtruth = groundtruth[option-1]
        mismatch[int((groundtruth_x-25)/5)][int(decimate-1)] = abs(test_mean-test_groundtruth)
        test_variance = summary['apriltag_output'][mismatch_option[option]]['variance']
        variance[int((groundtruth_x-25)/5)][int(decimate-1)] = test_variance


print(mismatch)

data = []
for i in range(0,4):
    data.append(
        go.Bar(
            x = decimate_list,
            y = mismatch[i],
            name = 'x='+str(15+i*5)+'cm'
        )
    )

symbol = ['square', 'circle', 'triangle-up', 'x' ]
for i in range(0,4):
    data.append(
        go.Scatter(
            x = decimate_list,
            y = variance[i],
            name = 'variance: x='+str(15+i*5)+'cm',
            yaxis = 'y2',
            mode = 'lines+markers',
            marker={
                'size': 10,
                'symbol': symbol[i]
            }
        )
    )


if(option == 3):
    y_axis = {
        'title':'mismatch(degree)',
        'titlefont':{
            'size':20
        }
    }
else:
    y_axis = {
        'title':'mismatch(cm)',
        'titlefont':{
            'size':20
        },
    }

layout = go.Layout(
    barmode='group',
    title = 'decimate-mismatch for ' + mismatch_option[option] +' (y=0, yaw=0)',
    titlefont = {
            'size':30
    },
    xaxis = {
        'title':'decimate',
        'titlefont':{
            'size':20
        }
    },
    yaxis = y_axis,
    yaxis2 = {
        'title':'variance',
        'titlefont':{
            'size':20
        },
        'overlaying':'y',
        'side':'right'
    }
)

fig = go.Figure(data=data, layout=layout)
py.plot(fig, filename='decimate-mismatch for ' + mismatch_option[option] +' (y=0, yaw=0)')

