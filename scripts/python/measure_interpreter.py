import matplotlib.pyplot as plt
import pandas as pd

class FIRFilter:
  def __init__(self,weights):
    self.weights = weights
    self.prev_values = [0]*len(weights)

  def filter(self, value,dt):
    #simple traditionla 
    self.prev_values = self.prev_values[1:] + [value]
    return sum([a*b for a,b in zip(self.weights,self.prev_values)])
  
class AverageFilter:
  def __init__(self, window_size):
    self.window_size = window_size
    self.prev_values = [0]*window_size

  def filter(self, value,dt):
    self.prev_values = self.prev_values[1:] + [value]
    return sum(self.prev_values) / self.window_size
  
class AlfaBetaFilter:
  def __init__(self, alfa, beta):
    self.alfa = alfa
    self.beta = beta
    self.prev_value = 0
    self.prev_velocity = 0
    self.ypri = 0
    self.ypost = 0
    self.vpri = 0
    self.vpost = 0

  def filter(self, value, dt):
    self.ypri = self.ypost + dt*self.vpost
    self.vpri = self.vpost
    self.ypost = self.ypri + self.alfa*(value - self.ypri)
    self.vpost = self.vpri + self.beta*(value - self.ypri)/dt
    return self.ypost
  
class MedianFilter:
  def __init__(self, window_size):
    self.window_size = window_size
    self.prev_values = [0]*window_size

  def filter(self, value,dt):
    self.prev_values = self.prev_values[1:] + [value]
    return sorted(self.prev_values)[self.window_size//2]

def read_ecndeor_vel_pos(file):
  p = pd.read_csv(file, header=None)
  arrays = p.to_numpy()
  read_angles = []
  read_angle_prev = []
  encoder_velocity = []
  dt_ = []
  tim = 0.0
  for a in arrays:
    time = float(a[0].split(';')[0])
    ang = float(a[0].split(';')[1])
    prev_ang = float(a[0].split(';')[2])
    velo = float(a[0].split(';')[3])
    read_angles.append(ang)
    read_angle_prev.append(prev_ang)
    dt_.append(time)
    encoder_velocity.append(velo)
  return read_angles, read_angle_prev, dt_, encoder_velocity

def calcualte_velocity(read_angles ,read_angle_prev, dt_):
  calc_velocity = []
  time_read = []
  ti = 0
  # filtr = FIRFilter([0.1,0.15,0.15,0.2,0.4])
  # filtr = AlfaBetaFilter(0.2,0.01)
  filtr = MedianFilter(5)
  for i in range(len(read_angles)):
    x = read_angles[i]
    ang_p = read_angle_prev[i]
    dt = dt_[i]
    ti += dt
    time_read.append(ti)
    vel = (x - ang_p)/ (dt)

    vel = filtr.filter(vel,dt)
    calc_velocity.append(vel)
  return calc_velocity, time_read

def plot_velocity(calc_velocity, encoder_velo,time_read):
  plt.plot(time_read, calc_velocity, label='calculated velocity', color='blue')
  plt.plot(time_read, encoder_velo, label='encoder velocity', linestyle='dashed', color='red')
  # plot avarage velocity
  avg = sum(calc_velocity) / len(calc_velocity)
  plt.axhline(y=avg, color='g', linestyle='-', label='average velocity')
  plt.legend()
  plt.show()

def main():
  file = 'measures/data_enc_vel.csv'
  read_angles, read_angle_prev,dt_,velo = read_ecndeor_vel_pos(file)
  calc_velocity, time_read = calcualte_velocity(read_angles, read_angle_prev, dt_)
  plot_velocity( calc_velocity,velo ,time_read)
  pass

def main2():
  file = 'measures/data_enc_pos_timc.csv'
  #read pos and time
  file = open(file, 'r')
  lines = file.readlines()
  read_angles = []
  read_time = []
  for l in lines:
    read_time.append(float(l.split(';')[1]))
    read_angles.append(float(l.split(';')[0]))
  file.close()

  #calculate velocity
  calc_velocity = []
  default_vel = []
  filtr = FIRFilter([0.1,0.6,0.2])
  # filtr = AlfaBetaFilter(0.2,0.01)
  # filtr = MedianFilter(5)
  pref=0
  for i in range(1,len(read_angles)):
    x = read_angles[i]
    dt = read_time[i] - read_time[i-1]
    vel = (x - read_angles[i-1])/ dt
    default_vel.append(vel)
    vel = 0.1*vel + 0.9*pref
    pref = vel
    # vel = filtr.filter(vel,dt)
    calc_velocity.append(vel)  

  #plot velocity
  plt.plot(read_time[1:], calc_velocity, label='calculated velocity', color='blue')
  # plt.plot(read_time[1:], default_vel, label='default velocity', color='purple')
  # plt.plot(read_time, read_angles, label='encoder position', linestyle='dashed', color='red')
  # plot avarage velocity
  avg = sum(calc_velocity) / len(calc_velocity)
  plt.axhline(y=avg, color='g', linestyle='-', label='average velocity')
  plt.legend()
  plt.show()

def main3():
  file = 'measures/data_pos_vel_tim.csv'
  #read pos and time
  file = open(file, 'r')
  lines = file.readlines()
  read_angles = []
  read_vel = []
  read_time = []
  set_vel = []
  for l in lines:
    set_vel.append(float(l.split(';')[3]))
    read_time.append(float(l.split(';')[2]))
    read_vel.append(float(l.split(';')[1]))
    read_angles.append(float(l.split(';')[0]))
  file.close()

  set_vel = set_vel[70:]
  read_time = read_time[70:]
  read_vel = read_vel[70:]
  read_angles = read_angles[70:]



  #calculate velocity
  calc_velocity = []
  ma = AverageFilter(60)
  mf= AlfaBetaFilter(0.1,0.01)
  # take every n point to the filter
  n = 5
  time_filter = [ read_time[i] for i in range(0,len(read_time),n)]
  pos_filter = [ read_angles[i] for i in range(0,len(read_angles),n)]

  for i in range(1,len(time_filter)):
    x = pos_filter[i]
    dt = time_filter[i] - time_filter[i-1]
    vel = (x - pos_filter[i-1])/ dt
    vel = ma.filter(vel,dt)
    # vel =  mf.filter(vel,dt)
    calc_velocity.append(vel)
  time_filter = time_filter[1:]

  #plot read_vel but only existing points
  plt.plot(read_time, read_vel, label='read velocity', color='blue', marker='o')
  #plot set velocity
  plt.plot(read_time, set_vel, label='calculated mediana', color='red')
  #plot calculated velocity
  plt.plot(time_filter, calc_velocity, label='calculated filtered', color='green')
  plt.legend()
  plt.show()



if __name__ == '__main__':
  # main()
  main3()