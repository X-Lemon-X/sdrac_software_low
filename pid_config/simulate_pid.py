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
    kp = 0.01
    ki = 0.01
    kd = 0
    dt = 0.001
    disturbance = 0.00001
    target_position = 3.14
    current_position = 0
    current_velocity = 0
    target_velocity = 0.3
    pid = PID(kp, ki, kd, dt)


    veloci = []
    tim = []
    pos = []

    for i in np.arange(0, 1, dt):
      current_velocity, pos_V = pid.update(current_position, target_position, current_velocity, target_velocity)
      current_position += current_velocity * (dt+disturbance)
      veloci.append(current_velocity)
      tim.append(i)
      pos.append(pos_V)

    return tim, veloci, pos

if __name__ == "__main__":
  tim , vel, pos = simulate_pid()
  #diplay chart the velocity
  plt.plot(tim, vel, 'b')
  plt.plot(tim, pos, 'r')
  plt.xlabel('Time')
  plt.ylabel('Velocity')
  plt.legend(['Velocity', 'Position'])
  plt.show()
