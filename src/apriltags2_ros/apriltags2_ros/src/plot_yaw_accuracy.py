import plotly.plotly as py
import plotly.graph_objs as go
import os
from ruamel import yaml
import numpy as np
import math


option = 3 # select from [1,2,3]
mismatch_option = {1:'x (cm)', 2:'y (cm)', 3:'oz (degree)'}  # test option;  oz represents yaw

groundtruth = {'x (cm)':[], 'y (cm)':[], 'oz (degree)':[]} # oz represents yaw
mismatch = [] #mismatch 
stdv = [] #standerd Deviation 
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
        test_mean = summary['apriltag_output'][mismatch_option[option]]['mean'] 
        test_stdv = math.sqrt(summary['apriltag_output'][mismatch_option[option]]['variance'])
        test_groundtruth = groundtruth[mismatch_option[option]][-1]
        # groundtruth[x] is the distance between camera and apriltag, we need to add 10 cm to get distance between duckiebot and apriltag
        if option == 1: 
            test_groundtruth = np.array(test_groundtruth) + 10
        mismatch.append(abs(test_mean-test_groundtruth))
        stdv.append(test_stdv)
        text.append('mismatch: ' + str(abs(test_mean-test_groundtruth)) + '<br>stdv: ' + str(test_stdv))


size = []
if option == 3:   # size represents standerd Deviation when plotting for mismatch of yaw
    size = np.array(stdv) * 3 + 10
else:    
    size = np.array(stdv) * 200 + 10 

data = [
    {
        'x': groundtruth['x (cm)'],
        'y': groundtruth['oz (degree)'],
        'text': text,
        'mode': 'markers',
        'marker': {
            'color': mismatch,  #mismatch
            'size': size, 
            'showscale': True
        }
    }
]

#add title and axis name
layout = go.Layout(
    title = 'yaw-mismatch for ' + mismatch_option[option] +' (y=0)',
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
        'title':'yaw(degree)',
        'titlefont':{
            'size':20
        }
    }
)

fig = go.Figure(data=data, layout=layout)

file_name = 'yaw-mismatch for ' + mismatch_option[option] +' (y=0)'
py.plot(fig, filename=file_name)

