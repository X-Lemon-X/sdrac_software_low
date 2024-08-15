import serial
import serial
import json
import threading
import serial.tools.list_ports as list_ports
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import tabulate
import numpy as np
data_serial = []

# Function to search for the serial port with the SDRAC board
def find_serial_port():
  ports = list_ports.comports()
  for port in ports:
    if "SDRAC" in port.description:
      return port.device
  return None

# Function to read data from the serial port and convert it to a plot
def read_serial_data(port):
  ser = serial.Serial(port)
  try:
    print(f"Time;Pos;Velocity")
    while True:
      line = ser.readline().decode().strip()
      try:
        json_data = json.loads(line)
        time = json_data['time']
        pos = json_data['msg']['Pos']
        velocity = json_data['msg']['Vel']
        print(f"{time};{pos};{velocity}")
        # Process the JSON data and print it in a nice-looking format
        # json_str = json.dumps(json_data, indent=2)
        # print(json_str)
        # Process the JSON data and update the chart
        # data_serial.append(json_data)
        # # Print the JSON data in a table format
        # table = tabulate.tabulate([[json_data["time"], json_data["level"], json_data["ver"], json_data["id"]]], headers=["time", "level", "ver", "id"], tablefmt="fancy_grid")
        # print(table)
        # # table = tabulate.tabulate([json_data["time"], json_data["level"], json_data["ver"], json_data["id"]], headers=["time", "level",'ver','id'], tablefmt="fancy_grid")
        # da = json_data['msg']
        # listvalues = []
        # listkeys = []
        # for key in da.keys():
        #   if(key == 'Errs'):
        #     for key2 in da[key].keys():
        #       listvalues.append(da[key][key2])
        #       listkeys.append(key2)
        #   else:
        #     listvalues.append(da[key])
        #     listkeys.append(key)
        # table2 = tabulate.tabulate( [listvalues],headers=listkeys, tablefmt="fancy_grid")
        # print(table)
        # print(table2)
      except json.JSONDecodeError:
        pass
      except ValueError:
        pass
  except KeyboardInterrupt:
    print("Exiting...")
    ser.close()


fig, ax = plt.subplots()
line,=  ax.plot([], [])

def update_plot(frame,line):
  if(len(data_serial) == 0):
    return line,

  if len(data_serial) >= 500:
    data = data_serial[-500:]
  else:
    data = data_serial

  time = [float(d["time"]) for d in data]
  level = [float(d['msg']["Pos"]) for d in data]
  x = np.linspace(0, max(time), len(time))
  # y = np.interp(x, time, level)
  line, = ax.plot(time, level, 'r-')
  # line.set_data(x, level)
  return line,


# x = np.linspace(0, 10)
# y = np.sin(x)

# fig, ax = plt.subplots()
# line, = ax.plot(x, y)

# def update(num, x, y, line):
#     line.set_data(x[:num], y[:num])
#     return line,


# Main function
def main():

  port = find_serial_port()
  if port:
    # Start a separate thread for reading the data and updating the chart
    thread = threading.Thread(target=read_serial_data, args=(port,))
    thread.start()
    # Plot the data
    # ani = FuncAnimation(fig, update_plot, 500, interval=50,fargs=[line], blit=True)
    # plt.show()
  else:
    print("SDRAC board not found.")

if __name__ == "__main__":
  main()