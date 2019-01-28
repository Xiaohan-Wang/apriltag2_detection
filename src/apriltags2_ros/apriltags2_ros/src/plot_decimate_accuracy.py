import plotly.plotly as py
from plotly import tools
import plotly.graph_objs as go
import os
from ruamel import yaml
import numpy as np
import copy
import math

option = 3 # select from [1,2,3]
mismatch_option = {1:'x (cm)', 2:'y (cm)', 3:'oz (degree)'}  # test option;  oz represents yaw

#summary file path
summary_path = '/home/xiaohan/duckietown/apriltag2_detection/src/apriltags2_ros/apriltags2_ros/test_data/decimate=1-12/summary' 

decimate_list = [1, 2, 3, 4, 6, 8, 12]
x = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120]
decimate_pos = {
    1: 0,
    2: 1,
    3: 2,
    4: 3,
    6: 4,
    8: 5,
    12: 6
}
decimate_dis = {
    0: 120,
    1: 120,
    2: 80,
    3: 60,
    4: 40,
    5: 30,
    6: 20
}
# 7*12 array. Each row corresponds to one position (x=10, 20, ... , 120) with different decimate
mismatch = [[0 for i in range(12)] for j in range(7)]  
variance = [[0 for i in range(12)] for j in range(7)]  

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
        mismatch[decimate_pos[decimate]][int(groundtruth_x/10)-2] = abs(test_mean-test_groundtruth)
        test_stdv = math.sqrt(summary['apriltag_output'][mismatch_option[option]]['variance'])
        variance[decimate_pos[decimate]][int(groundtruth_x/10)-2] = test_stdv



data = []
for i in range(0,7):
    data.append(
        go.Bar(
            x = x[:decimate_dis[i]/10],
            y = mismatch[i][:decimate_dis[i]/10],
            name = 'mismatch: decimate='+str(decimate_list[i])
        )
    )

symbol = ['square', 'circle', 'triangle-up', 'x', 'hexagram-open', 'star-open', 'hourglass']
for i in range(0,7):
    data.append(
        go.Scatter(
            x = x[:decimate_dis[i]/10],
            y = variance[i][:decimate_dis[i]/10],
            name = 'variance: decimate='+str(decimate_list[i]),
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

