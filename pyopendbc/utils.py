import time


class Can:
  address = 0
  busTime = 0
  dat = None
  src = None

class Event():
  def __init__(self):
    self.msgtype = "can"
    self.valid = False
    self.logMonoTime = 0
    
    self.can: list[Can] = []
    self.sendcan: list[Can] = []

# Python implementation so we don't have to depend on boardd
def can_list_to_pycan(can_msgs, msgtype='can', logMonoTime=None):
  # dat = new_message(service=None)
  dat = Event()
  dat.logMonoTime = int(time.monotonic() * 1e9)
  dat.valid = False

  if logMonoTime is not None:
    dat.logMonoTime = logMonoTime

  for i, can_msg in enumerate(can_msgs):
    if msgtype == 'sendcan':
      dat.sendcan.append(Can())
      cc = dat.sendcan[-1]
    else:
      dat.can.append(Can())
      cc = dat.can[-1]

    cc.address = can_msg[0]
    cc.busTime = can_msg[1]
    cc.dat = bytes(can_msg[2])
    cc.src = can_msg[3]

  return [dat]