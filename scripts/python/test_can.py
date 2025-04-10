import can
import cantools
from pprint import pprint
import time
import sys

can_bus = can.interface.Bus('can0', bustype='socketcan', bitrate=1000000)

db = cantools.database.load_file('src/ariadna_constants/can_messages/output/can.dbc')

# koanrm_num = 1
# msg_status = db.get_message_by_name(f'konarm_{koanrm_num}_status')
# msg_set_pos = db.get_message_by_name(f'konarm_{koanrm_num}_set_pos')
# msg_get_pos = db.get_message_by_name(f'konarm_{koanrm_num}_get_pos')
 
# msge=msg_set_pos
# data = msg_set_pos.encode({"position": 1.0, "velocity": 0.7})
# msg = can.Message(arbitration_id=msge.frame_id, data=data, is_extended_id=False,is_remote_frame=False)

# data = msg_status.encode({"status": 1})
# msg = can.Message(arbitration_id=msg_status.frame_id, data=data, is_extended_id=False,is_remote_frame=True)

def get_pos(joint):
  msg_get_pos = db.get_message_by_name(f'konarm_{joint}_get_pos')
  data = msg_get_pos.encode({"position": 0.0, "velocity":0.0})
  msg = can.Message(arbitration_id=msg_get_pos.frame_id, data=data, is_extended_id=False,is_remote_frame=True)
  can_bus.send(msg)
  rec = can_bus.recv(0.1)
  if rec is not None:
    msg_rec = msg_get_pos.decode(rec.data)
    pprint(msg_rec)


def set_pos(joint,pos, vel,mode):

  msg_set_cm = db.get_message_by_name(f'konarm_{joint}_set_control_mode')
  data = msg_set_cm.encode({"control_mode": mode})
  msg = can.Message(arbitration_id=msg_set_cm.frame_id, data=data, is_extended_id=False,is_remote_frame=False)
  can_bus.send(msg)


  msg_set_pos = db.get_message_by_name(f'konarm_{joint}_set_pos')
  data = msg_set_pos.encode({"position": pos, "velocity": vel})
  msg = can.Message(arbitration_id=msg_set_pos.frame_id, data=data, is_extended_id=False,is_remote_frame=False)
  can_bus.send(msg)

arg1 = int(sys.argv[1]) if len(sys.argv) > 1 else 1

try:
  # while True:
    # get_pos(arg1)
    # get_pos(2)

    set_pos(arg1, 0, 0.2, 2)
    # time.sleep(2)
    # set_pos(1, 0.0, 0.7, 2)
    # time.sleep(0.5)
    # set_pos(1, 0.0, 0.7, 2)
    # time.sleep(0.5)
    # set_pos(1, 0.0, 0.7, 2)
  
  # set_pos(2, 0.0, 0.5)
  # set_pos(3, 0.0, 0.5)

except KeyboardInterrupt:
  pass

can_bus.shutdown()
