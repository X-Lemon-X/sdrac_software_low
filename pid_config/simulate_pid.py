import numpy as np
import matplotlib
import math
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt

class PID:
  def __init__(self, kp, ki, kd, dt,t1,t2):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.dt = dt
    self.t1 = t1
    self.t2= t2
    self.previous_position = 0
    self.time_sec=0

  def update(self, current_position, target_position, current_velocity, max_velocity,time_sec):
    self.time_sec=time_sec

    # error = target_position - current_position
    # pid_pos = (self.kp * error) + (self.kd * error / self.dt) + (self.ki * error * self.dt)
    error = target_position - current_position
    # next_position = (self.kp * error) + (error * self.kd / self.dt) + (error * self.ki * self.dt)
    # next_position = self.kp*(1-math.exp(-self.time_sec*self.t1))*(1-math.exp(-self.time_sec*self.t2))*target_position + (self.ki*error*self.dt)
    
    # if next_position > target_position:
    #   next_position = target_position
    
    velocity = (next_position - self.previous_position) / self.dt
    
    
    
    self.previous_position = next_position


    if(velocity > max_velocity):
      velocity = max_velocity
    if(velocity < -max_velocity):
      velocity = -max_velocity
    return velocity, next_position  
  
class Bsci_accelarator:
  def __init__(self, max_velocity, max_acceleration, dt,previous_position):
    self.max_velocity = max_velocity
    self.max_acceleration = max_acceleration
    self.dt = dt
    self.current_velocity = 0
    self.current_position = 0
    self.previous_position = previous_position
    self.previous_velocity = 0
    self.time_sec = 0

  def decrease_value(self, value, decrease_by)->float:
    if(value > 0):
      value -= decrease_by
    else:
      value += decrease_by
    return value

  def update(self, target_position,current_position):
    self.time_sec += self.dt
    self.current_position = current_position

    error = target_position - self.current_position
    deacel_time= abs(self.current_velocity/self.max_acceleration)
    deacceleretion_distance = abs((self.current_velocity * deacel_time)) - (0.5 * self.max_acceleration * deacel_time*deacel_time)
    deacceleretion_distance = abs(deacceleretion_distance)

    if(error > 0):
      if(error > deacceleretion_distance):      
        self.current_velocity += self.max_acceleration * self.dt
      else:
        self.current_velocity = self.decrease_value(self.current_velocity,self.max_acceleration * self.dt)
    else:
      if(abs(error) > deacceleretion_distance):      
        self.current_velocity -= self.max_acceleration * self.dt
      else:
        self.current_velocity = self.decrease_value(self.current_velocity,self.max_acceleration * self.dt)
    

    # if(error > deacceleretion_distance):      
    #   if(error > 0):
    #     self.current_velocity += self.max_acceleration * self.dt
    #   else:
    #     self.current_velocity -= self.max_acceleration * self.dt
    # else:
    #   if(error > 0):
    #     self.current_velocity -= self.max_acceleration * self.dt
    #   else:
    #     self.current_velocity -= self.max_acceleration * self.dt

    if(self.current_velocity > self.max_velocity):
      self.current_velocity = self.max_velocity
    elif(self.current_velocity < -self.max_velocity):
      self.current_velocity = -self.max_velocity

    # ratio_curent = 0.99
    # self.current_velocity = (self.previous_velocity * (1-ratio_curent)) + (self.current_velocity * ratio_curent)
    # self.previous_velocity = self.current_velocity

    if(abs(error) < 0.001):
      self.current_velocity = 0

    self.current_position += self.current_velocity * self.dt
    return self.current_velocity, self.current_position

  def set_kp(self, kp):
    self.kp = kp
  
  def set_ki(self, ki):
    self.ki = ki

  def set_kd(self, kd):
    self.kd = kd
  
  def set_dt(self, dt):
    self.dt = dt

def simulate_pid():
    kp = 1.1
    ki = 0
    kd = 0.0
    dt = 0.005
    t1 = 0.4
    t2 = 0.5
    disturbance = 0.1
    current_position = 0
    current_velocity = 0
    target_velocity = 1.3
    pid = PID(kp, ki, kd, dt,t1,t2)
    bc = Bsci_accelarator(1.3, 2, dt,current_position)

    veloci = []
    tim = []
    pos = []
    target_pos=[]
    pos_V = current_position

    target_position = 4.72
    for i in np.arange(0, 10, dt):
      current_velocity, pos_V = bc.update(target_position, pos_V)
      veloci.append(current_velocity)
      tim.append(i)
      pos.append(pos_V)
      target_pos.append(target_position)

    target_position = 0
    for i in np.arange(10,20, dt):
      current_velocity, pos_V = bc.update(target_position, pos_V)
      veloci.append(current_velocity)
      tim.append(i)
      pos.append(pos_V)
      target_pos.append(target_position)

    target_position = 4.72
    for i in np.arange(20, 30, dt):
      current_velocity, pos_V = bc.update(target_position, pos_V)
      veloci.append(current_velocity)
      tim.append(i)
      pos.append(pos_V)
      target_pos.append(target_position)

    return tim, veloci, pos,target_pos

if __name__ == "__main__":
  tim , vel, pos, target_pos = simulate_pid()
  #diplay chart the velocity
  plt.plot(tim, vel, 'b')
  plt.plot(tim, pos, 'r')
  plt.plot(tim, target_pos, 'g')
  # Check target position for changes and insert vertical lines
  previous_target = target_pos[0]
  for target in target_pos:
    if target != previous_target:
      plt.axvline(x=tim[target_pos.index(target)], color='gray', linestyle='--')
      previous_target = target
  plt.xlabel('Time')
  plt.ylabel('Pos [rad]')
  plt.legend(['Velocity','Position',"target_pos"])
  plt.show()
