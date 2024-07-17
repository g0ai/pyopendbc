from .common_dbc import Signal, SignalPackValue, Msg
from .dbc import dbc_lookup

import numpy as np

IULL = np.uint64(1)

class CANPacker:
  def __init__(self, dbc_name):
    self.dbc = None
    self.signal_lookup = {} # (0, ""), Signal())
    self.counters = {}

    self.dbc = dbc_lookup(dbc_name)
    # assert(dbc)

    for msg in self.dbc.msgs:
      for sig in msg.sigs:
        self.signal_lookup[(msg.address, sig.name)] = sig # std::make_pair

  # std::vector<uint8_t> CANPacker::pack(uint32_t address, const std::vector<SignalPackValue> &signals) {
  def _pack(self, address, signals):
    ok = address in self.dbc.addr_to_msg
    if not ok:
      print("undefined address %d", address)
      return
    msg_it = self.dbc.addr_to_msg[address]

    ret = [np.uint8(0) for i in range(msg_it.size)]

    # sig = Signal()
    # sig.msb = 0
    # sig.lsb = 7
    # sig.size = 8
    # ival = 5
    # set_value(ret, sig, ival)
    # return ret

    # set all values for all given signal/value pairs
    counter_set = False
    for sigval in signals:
      ok = (address, sigval.name) in self.signal_lookup
      # print(self.signal_lookup)
      if not ok:
        # TODO: do something more here. invalid flag like CANParser?
        print(f"undefined signal {sigval.name} - {address}\n")
        continue
      # else:
      #   print(self.signal_lookup)
      #   print("ok")

      sig = self.signal_lookup[(address, sigval.name)]
      # sig = sig_it

      # print("0000 ", sigval.value, sig.offset, sig.factor)
      ival = np.int64(round((sigval.value - sig.offset) / sig.factor))
      if ival < 0:
        ival = (IULL << sig.size) + ival
      
      # print(address, sigval.name, sigval.value)
      # print(address, sigval.name, sigval.value)
      set_value(ret, sig, ival)
      # print(ret)

      # exit()

      if sigval.name == "COUNTER":
        self.counters[address] = sigval.value
        counter_set = True

    # set message counter
    ok = (address, "COUNTER") in self.signal_lookup
    # sig_it_counter = []
    if not counter_set and ok:
      # sig = sig_it_counter.second
      sig = self.signal_lookup[(address, "COUNTER")]

      if address not in self.counters:
        self.counters[address] = 0

      set_value(ret, sig, self.counters[address])
      self.counters[address] = (self.counters[address] + 1) % (1 << sig.size)
    

    # set message checksum
    ok = (address, "CHECKSUM") in self.signal_lookup
    if ok:
      sig = self.signal_lookup[(address, "CHECKSUM")]
      if sig.calc_checksum is not None:
        checksum = sig.calc_checksum(address, sig, ret)
        set_value(ret, sig, checksum)
      
    return ret

  

  # This function has a definition in common.h and is used in PlotJuggler
  def lookup_message(self, address):
    return self.dbc.addr_to_msg[address]
  

  def pack(self, addr, values):
    # cdef vector[SignalPackValue] values_thing
    # values_thing.reserve(len(values))
    values_thing = [] # [SignalPackValue() for i in range(len(values))]
    # cdef SignalPackValue spv

    for name, value in values.items():
      spv = SignalPackValue()
      spv.name = name #.encode("utf8")
      spv.value = value
      values_thing.append(spv)

    return self._pack(addr, values_thing)


  def make_can_msg(self, name_or_addr, bus, values):
    addr = 0
    m = Msg()
    if isinstance(name_or_addr, int):
      addr = name_or_addr
    else:
      try:
        m = self.dbc.name_to_msg[name_or_addr] # .encode("utf8")]
        addr = m.address
      except Exception: # IndexError
        # The C++ pack function will log an error message for invalid addresses
        print(">>>>>>>>>>>")
        print(self.dbc.name_to_msg, name_or_addr)
        pass

    val = self.pack(addr, values)
    val = bytes(val)
    # return [addr, 0, (<char *>&val[0])[:val.size()], bus]
    #          [[298, 0, b'\xdf\x80\x00\x02@\x00\x08\x00\x00\x00\x00\x00\x00\x00\x00\x00', 0]]
    # WRONGE ; [[298, 0, b'\xa4\x8b\x00\x00\x01\x00\x04\x00\x00\x00\x00\x00\x00\x00\x00\x00', 0]]
    return [addr, 0, val, bus]


# std::vector<uint8_t> &msg, const Signal &sig, int64_t ival
def set_value(msg, sig, ival):
  i = sig.lsb // 8
  bits = sig.size
  # USELESS ??
  if sig.size < 64:
    ival &= np.int64((IULL << sig.size) - 1)

  while (i >= 0 and i < len(msg) and bits > 0):
    shift = sig.lsb % 8 if (sig.lsb // 8) == i else 0
    size = min(bits, 8 - shift)

    msg[i] &= np.uint8(~(((IULL << size) - 1) << shift))
    msg[i] |= np.uint8((ival & np.int64((IULL << size) - 1)) << shift)

    bits -= size
    ival >>= size
    i = i+1 if sig.is_little_endian else i-1





