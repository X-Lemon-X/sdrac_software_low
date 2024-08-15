import pandas as pd
import sys

import matplotlib.pyplot as plt
import matplotlib.widgets as widgets

# Load data from CSV file
arg1 = sys.argv[1]
data = pd.read_csv(arg1,delimiter=';', header=None, names=['State','Time', 'Pos', 'Velocity'])

# Extract time, position, and velocity columns
state_table = data['State']
time_table = data['Time']
position_table = data['Pos']
velocity_table = data['Velocity']
# Convert variables to arrays
size_table = len(time_table) - 1
state = [0] * size_table
time = [0] * size_table 
position = [0] * size_table
velocity = [0] * size_table

for i in range(1,size_table+1):
  x = time_table[i]
  time[i-1] = float(x)
  position[i-1] = float(position_table[i])
  velocity[i-1] = float(velocity_table[i])
  if state_table[i] == "accelerating":
    state[i-1] = 1
  else:
    state[i-1] = 0 

# Plot the graph
fig = plt.figure()
ax = fig.subplots()
ax.plot(time, position, label='Position')
ax.plot(time, velocity, label='Velocity')
ax.plot(time, state, label='State')
ax.set_xlabel('Time')
ax.set_ylabel('Value')
ax.set_title('Data Visualization')
ax.legend()
ax.grid()

cursor = widgets.Cursor(ax, vertOn=True, horizOn=True, color='red', linewidth=1,useblit=True)
plt.show()
