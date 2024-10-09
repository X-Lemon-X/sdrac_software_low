import can
import cantools
from pprint import pprint
import time

can_bus = can.interface.Bus('can0', bustype='socketcan', bitrate=250000)

db = cantools.database.load_file('src/ariadna_constants/can_messages/output/can.dbc')

koanrm_num = 3
msg_status = db.get_message_by_name(f'konarm_{koanrm_num}_status')
msg_set_pos = db.get_message_by_name(f'konarm_{koanrm_num}_set_pos')
msg_get_pos = db.get_message_by_name(f'konarm_{koanrm_num}_get_pos')
 

data = msg_get_pos.encode({"position": 0, "velocity": 0.3})
msg_data=msg_get_pos
msg = can.Message(arbitration_id=msg_data.frame_id, data=data, is_extended_id=False,is_remote_frame=True)

# data = msg_status.encode({"status": 1})
# msg = can.Message(arbitration_id=msg_status.frame_id, data=data, is_extended_id=False,is_remote_frame=True)

try:
  while True:
    can_bus.send(msg)
    rec = can_bus.recv(0.1)
    if rec is not None:
      msg_rec = msg_data.decode(rec.data)
      pprint(msg_rec)    
    time.sleep(0.1)
except KeyboardInterrupt:
  pass

can_bus.shutdown()
