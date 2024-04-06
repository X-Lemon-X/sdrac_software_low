import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class AlphaBeta:
  def __init__(self) -> None:
    self.alfa = 0
    self.beta = 0
    self.ypris=0
    self.ypost =0
    self.vpri=0.0
    self.vpost=0.0
    self.prev_time=0.0
  
  def calculate(self, value, time):
    dt = abs(self.prev_time - time)
    self.ypri = self.ypost + dt*self.vpost
    self.vpri = self.vpost
    self.ypost = self.ypri + self.alfa*(value - self.ypri)
    self.vpost = self.vpri + self.beta*(value- self.ypri)/dt
    self.prev_time =  time
    # if self.ypost < 0:
    #   self.ypost = 0.01
    return self.ypost
  

# load csv file
file = 'measures/data 1000hz.csv'

p = pd.read_csv(file, header=None)
arrays = p.to_numpy()

ab = AlphaBeta()
ab.alfa = 0.01
ab.beta = 0.001

read_angles = []
read_velocity = []

calc_angles = []
calc_velocity = []

time_read = []

freq_read = []

ang_p=0.0
tim=0.0
for a in arrays:
  x = float(a[0].split(';')[0])
  time = float(a[0].split(';')[1])
  time_read.append(time)
  read_angles.append(x)
  read_velocity.append(time)
  an = ab.calculate(x, time)
  calc_angles.append(an)
  vel = (an - ang_p)/ (time - tim)
  freq_read.append(1/(time - tim))
  
  tim = time
  ang_p = an
  calc_velocity.append(vel)

#plot on a single graph calcualted velocity and read velocity
# plt.plot(time_read, read_velocity, label='read velocity')
# plt.plot(time_read, calc_velocity, label='calculated velocity')

#plot on a second graph calculated angle and read angle
# add circles for each point
plt.plot(time_read, read_angles, label='read angle', marker='o')
plt.plot(time_read, calc_angles, label='calculated angle', marker='o')

#dilpsay both graphs
plt.xlabel('Time')
plt.ylabel('Value')
plt.legend()
plt.show()
#calcayle avarage frequency
def avg(arr):
  return sum(arr)/len(arr)
print(avg(freq_read))



