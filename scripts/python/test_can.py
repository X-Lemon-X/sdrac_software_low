import can
import cantools
from pprint import pprint
import time

can_bus = can.interface.Bus('can0', bustype='socketcan', bitrate=250000)

db = cantools.database.load_file('../../src/ariadna_constants/can_messages/output/can.dbc')

msg_status = db.get_message_by_name('konarm_1_status')
msg_set_pos = db.get_message_by_name('konarm_1_set_pos')
msg_get_pos = db.get_message_by_name('konarm_1_get_pos')


data = msg_set_pos.encode({"position": 0, "velocity": 0.3})
msg = can.Message(arbitration_id=msg_set_pos.frame_id, data=data, is_extended_id=False)

# data = msg_status.encode({"status": 1})
# msg = can.Message(arbitration_id=msg_status.frame_id, data=data, is_extended_id=False,is_remote_frame=True)

try:
# while True:
    pprint(msg) 
    can_bus.send(msg)
    time.sleep(1)
except KeyboardInterrupt:
  pass
  
