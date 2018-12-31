import plotly.plotly as py
import plotly.graph_objs as go
import os
from ruamel import yaml
import numpy as np


option = 1 # select from [1,2,3]
mismatch_option = {1:'x (cm)', 2:'y (cm)', 3:'oz (degree)'}  # test option;  oz represents yaw

groundtruth = {'x (cm)':[], 'y (cm)':[], 'oz (degree)':[]} # oz represents yaw
mismatch = [] #mismatch 
variance = [] #variance

path_name = os.getcwd()
parent_path = os.path.dirname(path_name)
summary_path = os.path.join(parent_path, "test_result", "summary")

for filename in os.listdir(summary_path):
    groundtruth['x (cm)'].append(float(filename.split('_')[2])) 
    groundtruth['y (cm)'].append(float(filename.split('_')[3]))
    groundtruth['oz (degree)'].append(float(filename.split('_')[4]))

    with open(summary_path + '/' + filename, 'r') as f:
        summary = yaml.load(f.read())   
        test_mean = summary['apriltag_output'][mismatch_option[option]]['mean']
        test_variance = summary['apriltag_output'][mismatch_option[option]]['variance']
        test_groundtruth = groundtruth[mismatch_option[option]][-1]

        mismatch.append(abs(test_mean-test_groundtruth))
        variance.append(test_variance)

# print('----------------------------------------------------')
# print(mismatch)
# print(max(mismatch)-min(mismatch))
# print('----------------------------------------------------')

# color = (np.array(mismatch)-min(mismatch))/(max(mismatch)-min(mismatch))*255
# #TODO:how to resize size?
# size = np.array(variance)/(min(variance)+1e-8)*10

data = [
    {
        'x': groundtruth['x (cm)'],
        'y': groundtruth['y (cm)'],
        'mode': 'markers',
        'marker': {
            # 'color': [120, 125, 130, 135, 140, 145],
            # 'size': [15, 30, 55, 70, 90, 110],
            # 'color': color,  #mismatch
            # 'size': size,  #variance
            'color': 150,  #mismatch
            'size': 50,  #variance
            'showscale': True
        }
    }
]

file_name = "mismatch - " + mismatch_option[option] + " - yaw:" + str(groundtruth['oz (degree)'][0])
py.plot(data, filename=file_name)

