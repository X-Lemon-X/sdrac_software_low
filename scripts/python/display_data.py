import pandas as pd
import sys

import matplotlib.pyplot as plt
import matplotlib.widgets as widgets

# Load data from CSV file
# arg1 = sys.argv[1]
arg1 = "data.csv"
data = pd.read_csv(arg1,delimiter=';', header=None, names=['Time', 'Pos', 'Velocity'])

# Extract time, position, and velocity columns
# state_table = data['State']
time_table = data['Time']
position_table = data['Pos']
velocity_table = data['Velocity']
# Convert variables to arrays
size_table = len(time_table) - 1
# state = [0] * size_table
time = [0] * size_table 
position = [0] * size_table
velocity = [0] * size_table

time_begin = float(time_table[1])/1000.0
coubnt_to_velocity_sample = 0
for i in range(1,size_table+1):
  x = time_table[i]
  time[i-1] = float(x)/1000.0 - time_begin
  position[i-1] = float(position_table[i])
  
  # if coubnt_to_velocity_sample == 10:
  #   dt = time[i-1] - time[i-11]
  #   velocity[i-1] = (position[i-1] - position[i-11]) / dt
  #   velocity[i-1] = float(velocity_table[i])
  #   coubnt_to_velocity_sample = 0
  # coubnt_to_velocity_sample += 1

  velocity[i-1] = float(velocity_table[i])
  # if state_table[i] == "accelerating":
  #   state[i-1] = 1
  # else:
  #   state[i-1] = 0 

# Plot the graph
fig = plt.figure()
ax = fig.subplots()
ax.plot(time, position, label='Position', color='blue', linewidth=1, marker='o')
calculate_velocity = [0] * size_table
for i in range(1,size_table):
  dt = time[i] - time[i-1]
  calculate_velocity[i] = (position[i] - position[i-1]) / dt
# ax.plot(time, calculate_velocity, label='Calculate Velocity', color='green', linewidth=1, marker='o')

velocitry_final = (position[-1] - position[0])/ (time[-1] - time[0])
print(f"Final velocity: {velocitry_final}")


# ax.plot(time, velocity, label='Velocity', color='red', linewidth=1, marker='o')
# ax.plot(time, state, label='State')
ax.set_xlabel('Time')
ax.set_ylabel('Value')
ax.set_title('Data Visualization')
ax.legend()
ax.grid()

cursor = widgets.Cursor(ax, vertOn=True, horizOn=True, color='red', linewidth=1,useblit=True)
plt.show()
