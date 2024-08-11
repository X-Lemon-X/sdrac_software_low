import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt

class PID:
  def __init__(self, kp, ki, kd, dt):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.dt = dt
    self.previous_position = 0

  def update(self, current_position, target_position, current_velocity, target_velocity):
    error = target_position - current_position
    pid_pos = (self.kp * error) + (self.kd * error / self.dt) + (self.ki * error * self.dt)
    return (pid_pos-current_position) / self.dt, pid_pos  
  
  def set_kp(self, kp):
    self.kp = kp
  
  def set_ki(self, ki):
    self.ki = ki

  def set_kd(self, kd):
    self.kd = kd
  
  def set_dt(self, dt):
    self.dt = dt

def simulate_pid():
    kp = 1.2
    ki = 0.001
    kd = 0.0000001
    dt = 0.0001
    disturbance = 0.00001
    target_position = 3.14
    current_position = 0
    current_velocity = 0
    target_velocity = 0.3
    pid = PID(kp, ki, kd, dt)


    veloci = []
    tim = []
    pos = []
    target_pos=[]
    pos_V = current_position
    for i in np.arange(0, 2, dt):
      # current_velocity, pos_V = pid.update(current_position, target_position, current_velocity, target_velocity)
      # current_position += current_velocity * (dt+disturbance)
      current_velocity, pos_V = pid.update(pos_V, target_position, current_velocity, target_velocity)
      veloci.append(current_velocity)
      tim.append(i)
      pos.append(pos_V)
      target_pos.append(target_position)
    return tim, veloci, pos,target_pos

if __name__ == "__main__":
  tim , vel, pos, target_pos = simulate_pid()
  #diplay chart the velocity
  # plt.plot(tim, vel, 'b')
  plt.plot(tim, pos, 'r')
  plt.plot(tim, target_pos, 'g')
  plt.xlabel('Time')
  plt.ylabel('Pos [rad]')
  plt.legend(['Position',"target_pos"])
  plt.show()
