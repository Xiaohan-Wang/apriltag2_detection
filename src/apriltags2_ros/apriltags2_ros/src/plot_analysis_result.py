import plotly.plotly as py
import plotly.graph_objs as go
import os
from ruamel import yaml
import numpy as np


option = 3 # select from [1,2,3]
mismatch_option = {1:'x (cm)', 2:'y (cm)', 3:'oz (degree)'}  # test option;  oz represents yaw

groundtruth = {'x (cm)':[], 'y (cm)':[], 'oz (degree)':[]} # oz represents yaw
mismatch = [] #mismatch 
variance = [] #variance
text = []

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
        if option == 1: # I forget to add 10cm to the distance between camera and apriltag
            test_groundtruth = np.array(test_groundtruth) + 10
        mismatch.append(abs(test_mean-test_groundtruth))
        variance.append(test_variance)
        text.append('mismatch: ' + str(abs(test_mean-test_groundtruth)) + '<br>variance: ' + str(test_variance))


# size represents variance for yaw
if option == 3:
    size = np.array(variance)*50000   #for yaw, we include the variance information in the plot
else:
    size = [10] * len(variance)  # for x and y, we do not include varinance information in the plots

# size = np.array(variance)
# print(size)

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

file_name = "mismatch - " + mismatch_option[option] + " - yaw:" + str(groundtruth['oz (degree)'][0])
py.plot(data, filename=file_name)

