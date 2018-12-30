import plotly.plotly as py
import plotly.graph_objs as go

for i in range(10,31,2):  # x:forward, 10cm-30cm
 	for j in range(-10,2,11):  # y:right  -10cm-10cm
 	




data = [
    {
        'x': [1, 3.2, 5.4, 7.6, 9.8, 12.5],
        'y': [1, 3.2, 5.4, 7.6, 9.8, 12.5],
        'mode': 'markers',
        'marker': {
            'color': [120, 125, 130, 135, 140, 145],  #mismatch
            'size': [15, 30, 55, 70, 90, 110],  #variance
            'showscale': True
        }
    }
]

py.iplot(data, filename='mismatch-x')
# py.iplot(data, filename='mismatch-y')
# py.iplot(data, filename='mismatch-z')
# py.iplot(data, filename='mismatch-ox')
# py.iplot(data, filename='mismatch-oy')
# py.iplot(data, filename='mismatch-oz')
