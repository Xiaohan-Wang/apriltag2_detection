import plotly.plotly as py
import plotly.graph_objs as go
import os
from ruamel import yaml
import numpy as np
import math


option = 1 # select from [1,2,3]
mismatch_option = {1:'x (cm)', 2:'y (cm)', 3:'oz (degree)'}  # test option;  oz represents yaw

groundtruth = {'x (cm)':[], 'y (cm)':[], 'oz (degree)':[]} # oz represents yaw
mismatch = [] #mismatch 
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
        test_mean = summary['apriltag_output'][mismatch_option[option]]['mean'] 
        test_variance = summary['apriltag_output'][mismatch_option[option]]['variance']
        test_groundtruth = groundtruth[mismatch_option[option]][-1]
        # groundtruth[x] is the distance between camera and apriltag, we need to add 10 cm to get distance between duckiebot and apriltag
        if option == 1: 
            test_groundtruth = np.array(test_groundtruth) + 10
        mismatch.append(abs(test_mean-test_groundtruth))
        variance.append(test_variance)
        text.append('mismatch: ' + str(abs(test_mean-test_groundtruth)) + '<br>variance: ' + str(test_variance))


size = []
if option == 3:   # size represents variance when plotting for mismatch of yaw
    for i in range(len(variance)):
        size.append(60 + math.log(variance[i], 10)*10)
else:    # for mismatch for x and y, we do not include varinance information in the plots
    size = [10] * len(variance)  

data = [
    {
        'x': groundtruth['x (cm)'],
        'y': groundtruth['y (cm)'],
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
    title = 'mismatch for ' + mismatch_option[option] + ' (yaw= '+ str(groundtruth['oz (degree)'][0]) + ')',
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

file_name = 'mismatch for ' + mismatch_option[option] + ' (yaw= '+ str(groundtruth['oz (degree)'][0]) + ')'
py.plot(fig, filename=file_name)

