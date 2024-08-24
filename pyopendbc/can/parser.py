from collections import defaultdict
import sys
import numbers
import numpy as np

# from ..utils import Event

from .common import MAX_BAD_COUNTER, CAN_INVALID_CNT
from .common_dbc import SignalType, SignalValue
from .dbc import dbc_lookup

IULL = np.uint64(1) # 1ULL

def get_raw_value(msg, sig):
  ret = 0

  i = sig.msb // 8
  bits = sig.size
  while (i >= 0 and i < len(msg) and bits > 0):
    lsb = sig.lsb if (sig.lsb // 8) == i else i*8
    msb = sig.msb if (sig.msb // 8) == i else (i+1)*8 - 1
    size = msb - lsb + 1

    d = np.uint64((np.uint8(msg[i]) >> (lsb - (i*8))) & ((IULL << size) - 1))
    # print("bits, size", bits, size)
    ret |= d << (bits - size)

    bits -= size
    i = i-1 if sig.is_little_endian else i+1
  return np.int64(ret)


class MessageState:
  def __init__(self):
    self.name = ""
    self.address = 0
    self.size = 0

    self.parse_sigs = []
    self.vals = []
    self.all_vals = []

    self.last_seen_nanos = 0
    self.check_threshold = 0

    self.counter = 0
    self.counter_fail = 0

    self.ignore_checksum = True
    self.ignore_counter = True

  # bool parse(uint64_t nanos, const std::vector<uint8_t> &dat);
  def parse(self, nanos, dat):
    # tmp_vals = [0.0 for i in range(len(self.parse_sigs))]
    tmp_vals = []
    checksum_failed = False
    counter_failed = False

    for i in range(len(self.parse_sigs)):
      sig = self.parse_sigs[i]


      tmp = get_raw_value(dat, sig)
      if (sig.is_signed):
        # tmp = tmp # - (IULL << sig.size if ((tmp >> (sig.size-1)) & 0x1) else 0)
        if ((tmp >> (sig.size-1)) & 0x1):
          tmp = np.int64(tmp)
          # print(sig.size, IULL, IULL << sig.size, type(tmp))
          # print()
        tmp -= IULL << sig.size if ((tmp >> (sig.size-1)) & 0x1) else 0
      

      # print(dat, sig.lsb, sig.type, tmp)

      #DEBUG("parse 0x%X %s -> %ld\n", address, sig.name, tmp);

      if not self.ignore_checksum:
        if sig.calc_checksum is not None and sig.calc_checksum(self.address, sig, dat) != tmp:
          checksum_failed = True

      if not self.ignore_counter:
        if sig.type == SignalType.COUNTER and not self.update_counter_generic(tmp, sig.size):
          counter_failed = True


      tmp_vals.append(tmp * sig.factor + sig.offset)

    # only update values if both checksum and counter are valid
    if (checksum_failed or counter_failed):
      print(f"0x{self.address} message checks failed, checksum failed {checksum_failed}, counter failed {counter_failed}")
      
      return False

    for i in range(len(self.parse_sigs)):
      self.vals[i] = tmp_vals[i]
      self.all_vals[i].append(self.vals[i])
    
    self.last_seen_nanos = nanos

    return True

  # bool update_counter_generic(int64_t v, int cnt_size);
  def update_counter_generic(self, v, cnt_size):
    if (((self.counter + 1) & ((1 << cnt_size) -1)) != v):
      self.counter_fail = min(self.counter_fail + 1, MAX_BAD_COUNTER)
      if (self.counter_fail > 1):
        print(f"0x{self.address} COUNTER FAIL #{self.counter_fail} -- {self.counter} -> {v}\n")
    elif (self.counter_fail > 0):
      self.counter_fail -= 1
    
    self.counter = v
    return self.counter_fail < MAX_BAD_COUNTER
    





""" CAN Parser"""
class cpp_CANParser:
# private:
#   const int bus;
#   kj::Array<capnp::word> aligned_buf;

#   const DBC *dbc = NULL;
#   std::unordered_map<uint32_t, MessageState> message_states;

# public:
#   bool can_valid = false;
#   bool bus_timeout = false;
#   uint64_t first_nanos = 0;
#   uint64_t last_nanos = 0;
#   uint64_t last_nonempty_nanos = 0;
#   uint64_t bus_timeout_threshold = 0;
#   uint64_t can_invalid_cnt = CAN_INVALID_CNT;

#   CANParser(int abus, const std::string& dbc_name,
#             const std::vector<std::pair<uint32_t, int>> &messages);
#   CANParser(int abus, const std::string& dbc_name, bool ignore_checksum, bool ignore_counter);
#   #ifndef DYNAMIC_CAPNP
#   void update_string(const std::string &data, bool sendcan);
#   void update_strings(const std::vector<std::string> &data, std::vector<SignalValue> &vals, bool sendcan);
#   void UpdateCans(uint64_t nanos, const capnp::List<cereal::CanData>::Reader& cans);
#   #endif
#   void UpdateCans(uint64_t nanos, const capnp::DynamicStruct::Reader& cans);
#   void UpdateValid(uint64_t nanos);
#   void query_latest(std::vector<SignalValue> &vals, uint64_t last_ts = 0);


  def __init__(self):
    self.bus = 0
    self.aligned_buf = [0 for i in range(1024)]

    self.dbc = None
    self.message_states = {} # std::unordered_map<uint32_t, MessageState> 

    self.can_valid = False
    self.bus_timeout = False
    self.first_nanos = 0
    self.last_nanos = 0
    self.last_nonempty_nanos = 0
    self.bus_timeout_threshold = 0
    self.can_invalid_cnt = CAN_INVALID_CNT


  # int abus, const std::string& dbc_name, const std::vector<std::pair<uint32_t, int>> &messages
  def CANParser3(self, abus, dbc_name, messages):
    self.bus = abus
    self.aligned_buf = [0 for i in range(1024)]

    self.dbc = dbc_lookup(dbc_name)
    # assert(dbc)

    bus_timeout_threshold = sys.maxsize

    for address, frequency in messages:
      # disallow duplicate message checks
      if address in self.message_states:
        print(f"Duplicate Message Check: {address}")
        raise RuntimeError

      state = MessageState()
      state.address = address
      # state.check_frequency = op.check_frequency,

      # msg is not valid if a message isn't received for 10 consecutive steps
      if (frequency > 0):
        state.check_threshold = int((1000000000 / frequency) * 10)

        # bus timeout threshold should be 10x the fastest msg
        bus_timeout_threshold = min(bus_timeout_threshold, state.check_threshold)

      msg = self.dbc.addr_to_msg[address]
      state.name = msg.name
      state.size = msg.size
      assert(state.size <= 64);  # max signal size is 64 bytes

      # track all signals for this message
      state.parse_sigs = msg.sigs
      state.vals.extend([0.0 for i in range(len(msg.sigs))])
      state.all_vals.extend([[0.0] for i in range(len(msg.sigs))])

      self.message_states[address] = state

  # # int abus, const std::string& dbc_name, bool ignore_checksum, bool ignore_counter
  # def CANParser4(self, abus, dbc_name, ignore_checksum, ignore_counter):
  #   # Add all messages and signals
  #   self.bus = abus

  #   dbc = dbc_lookup(dbc_name)
  #   # assert(dbc)

  #   for msg in dbc.msgs:
  #     state = MessageState()
  #     state.name = msg.name
  #     state.address = msg.address
  #     state.size = msg.size
  #     state.ignore_checksum = ignore_checksum
  #     state.ignore_counter = ignore_counter

  #     for sig in msg.sigs:
  #       state.parse_sigs.append(sig)
  #       state.vals.append(0)
  #       state.all_vals.append([])

  #     self.message_states[state.address] = state


#ifndef DYNAMIC_CAPNP
  # const std::string &data, bool sendcan
  def update_string(self, data, sendcan: bool): # data: Event
    # format for board, make copy due to alignment issues.
    # const size_t buf_size = (data.length() / sizeof(capnp::word)) + 1;
    # if (aligned_buf.size() < buf_size) {
    #   aligned_buf = kj::heapArray<capnp::word>(buf_size);
    # }
    # memcpy(aligned_buf.begin(), data.data(), data.length());

    # # extract the messages
    # capnp::FlatArrayMessageReader cmsg(aligned_buf.slice(0, buf_size));
    # cereal::Event::Reader event = cmsg.getRoot<cereal::Event>();

    # if (first_nanos == 0) {
    #   first_nanos = event.getLogMonoTime();
    # }
    last_nanos = data.logMonoTime

    cans = data.sendcan if sendcan else data.can
    self.UpdateCans(last_nanos, cans)

    self.UpdateValid(last_nanos)


  # const std::vector<std::string> &data, std::vector<SignalValue> &vals, bool sendcan
  def update_strings(self, data: list[str], vals: list[str], sendcan: bool):
    current_nanos = 0
    for d in data:
      self.update_string(d, sendcan)
      if (current_nanos == 0):
        current_nanos = self.last_nanos
    self.query_latest(vals, current_nanos)
  

  # uint64_t nanos, const capnp::List<cereal::CanData>::Reader& cans
  def UpdateCans(self, nanos: int, cans):
    #DEBUG("got %d messages\n", cans.size());
    bus_empty = True

    # parse the messages
    for cmsg in cans:
      if cmsg.src != self.bus:
        # DEBUG("skip %d: wrong bus\n", cmsg.getAddress());
        continue
      bus_empty = False

      ok = cmsg.address in self.message_states
      if not ok:
        # DEBUG("skip %d: not specified\n", cmsg.getAddress());
        continue
      
      
      state_it = self.message_states[cmsg.address]

      dat = cmsg.dat

      if (len(dat) > 64):
        print(f"got message longer than 64 bytes: 0x{cmsg.address} {len(dat)}u\n")
        continue

      # TODO: this actually triggers for some cars. fix and enable this
      #if (dat.size() != state_it->second.size) {
      #  DEBUG("got message with unexpected length: expected %d, got %zu for %d", state_it->second.size, dat.size(), cmsg.getAddress());
      #  continue;
      #}

      # TODO: can remove when we ignore unexpected can msg lengths
      # make sure the data_size is not less than state_it->second.size
      # size_t data_size = max(len(dat.size(), state_it->second.size);
      # std::vector<uint8_t> data(data_size, 0);
      # memcpy(data.data(), dat.begin(), dat.size());
      # data = [0 for i in range(len(dat))]
      # cmsg.data abc
      
      state_it.parse(nanos, dat)

      # """

      # self.message_states[cmsg.address] = state_it

    # update bus timeout
    if not bus_empty:
      self.last_nonempty_nanos = nanos
    
    self.bus_timeout = (nanos - self.last_nonempty_nanos) > self.bus_timeout_threshold
  #endif

  # # uint64_t nanos, const capnp::DynamicStruct::Reader& cmsg
  # def UpdateCans():
  #   # assume message struct is `cereal::CanData` and parse
  #   assert(cmsg.has("address") && cmsg.has("src") && cmsg.has("dat") && cmsg.has("busTime"));

  #   if (cmsg.get("src").as<uint8_t>() != bus) {
  #     DEBUG("skip %d: wrong bus\n", cmsg.get("address").as<uint32_t>());
  #     return;
  #   }

  #   auto state_it = message_states.find(cmsg.get("address").as<uint32_t>());
  #   if (state_it == message_states.end()) {
  #     DEBUG("skip %d: not specified\n", cmsg.get("address").as<uint32_t>());
  #     return;
  #   }

  #   auto dat = cmsg.get("dat").as<capnp::Data>();
  #   if (dat.size() > 64) return; # shouldn't ever happen
  #   std::vector<uint8_t> data(dat.size(), 0);
  #   memcpy(data.data(), dat.begin(), dat.size());
  #   state_it->second.parse(nanos, data);
  # }

  # uint64_t nanos
  def UpdateValid(self, nanos: int):
    show_missing = (self.last_nanos - self.first_nanos) > 8e9

    _valid = True
    _counters_valid = True
    for kv in self.message_states:
      state = self.message_states[kv] # state = kv.second;

      if (state.counter_fail >= MAX_BAD_COUNTER):
        _counters_valid = False

      missing = state.last_seen_nanos == 0
      timed_out = (nanos - state.last_seen_nanos) > state.check_threshold
      if (state.check_threshold > 0 and (missing or timed_out)):
        if (show_missing and  not self.bus_timeout):
          if (missing):
            print(f"0x{state.address} '{state.name}' NOT SEEN")
          elif (timed_out):
            print(f"0x{state.address} '{state.name}' TIMED OUT")
        
        _valid = False
    
    self.can_invalid_cnt = 0 if _valid else (self.can_invalid_cnt + 1)
    self.can_valid = (self.can_invalid_cnt < CAN_INVALID_CNT) and _counters_valid

  # std::vector<SignalValue> &vals, uint64_t last_ts
  def query_latest(self, vals: list[SignalValue], last_ts: int):
    if (last_ts == 0):
      last_ts = self.last_nanos
    
    for kv in self.message_states:
      state = self.message_states[kv] # state = kv.second;
      if last_ts != 0 and state.last_seen_nanos < last_ts:
        continue

      for i in range(len(state.parse_sigs)):
        # const Signal &sig = state.parse_sigs[i];
        sig = state.parse_sigs[i]
        v = SignalValue() # &v = vals.emplace_back();
        v.address = state.address
        v.ts_nanos = state.last_seen_nanos
        v.name = sig.name
        v.value = state.vals[i]
        v.all_values = state.all_vals[i]
        state.all_vals[i].clear()

        vals.append(v)



class CANParser:
  # cdef:
  #   cpp_CANParser *can
  #   const DBC *dbc
  #   vector[SignalValue] can_values
  #   vector[uint32_t] addresses

  # cdef readonly:
  #   dict vl
  #   dict vl_all
  #   dict ts_nanos
  #   string dbc_name

  def __init__(self, dbc_name, messages, bus=0):
    self.dbc_name = dbc_name
    self.dbc = dbc_lookup(dbc_name)
    if not self.dbc:
      raise RuntimeError(f"Can't find DBC: {dbc_name}")

    self.vl = {}
    self.vl_all = {}
    self.ts_nanos = {}

    self.can_values: list[SignalValue] = []
    self.addresses: list[int] = []

    # Convert message names into addresses and check existence in DBC
    message_v = [] # cdef vector[pair[uint32_t, int]] message_v
    for i in range(len(messages)):
      c = messages[i]
      try:
        m = self.dbc.addr_to_msg[c[0]] if isinstance(c[0], numbers.Number) else self.dbc.name_to_msg[c[0]]
      except IndexError:
        raise RuntimeError(f"could not find message {repr(c[0])} in DBC {self.dbc_name}")

      address = m.address
      message_v.append((address, c[1]))
      self.addresses.append(address)

      name = m.name #.decode("utf8")
      self.vl[address] = {}
      self.vl[name] = self.vl[address]
      self.vl_all[address] = defaultdict(list)
      self.vl_all[name] = self.vl_all[address]
      self.ts_nanos[address] = {}
      self.ts_nanos[name] = self.ts_nanos[address]

    self.can = cpp_CANParser()
    self.can.CANParser3(bus, dbc_name, message_v)
    # self.update_strings([])

  def __dealloc__(self):
    if self.can:
      del self.can

  def update_strings(self, strings, sendcan=False):
    for address in self.addresses:
      self.vl_all[address].clear()

    new_vals: list[SignalValue] = []
    cur_address = -1
    # vl = {}
    # vl_all = {}
    # ts_nanos = {}
    # updated_addrs = set()

    # print(strings)
    # exit()
    self.can.update_strings(strings, new_vals, sendcan)
    # print(">>>>\n\n\n", [n.name for n in new_vals])
    # cdef vector[SignalValue].iterator it = new_vals.begin()
    # cdef SignalValue* cv
    for cv in new_vals:
      # cv = &deref(it)
      # Check if the address has changed
      if cv.address != cur_address:
        cur_address = cv.address
        vl = self.vl[cur_address]
        vl_all = self.vl_all[cur_address]
        ts_nanos = self.ts_nanos[cur_address]
        # updated_addrs.add(cur_address)

      # Cast char * directly to unicode
      cv_name = cv.name
      vl[cv_name] = cv.value
      vl_all[cv_name] = cv.all_values
      ts_nanos[cv_name] = cv.ts_nanos
      # preinc(it)
    
    
    del new_vals
    # del vl, vl_all, ts

    # del vl, vl_all, ts_nanos

    # return updated_addrs

  @property
  def can_valid(self):
    return self.can.can_valid

  @property
  def bus_timeout(self):
    return self.can.bus_timeout


class CANDefine():
#   cdef:
#     const DBC *dbc

#   cdef public:
#     dict dv
#     string dbc_name

  def __init__(self, dbc_name):
    self.dbc_name = dbc_name
    self.dbc = dbc_lookup(dbc_name)
    if not self.dbc:
      raise RuntimeError(f"Can't find DBC: '{dbc_name}'")

    dv = defaultdict(dict)

    for i in range(len(self.dbc.vals)):
      val = self.dbc.vals[i]

      sgname = val.name#.decode("utf8")
      def_val = val.def_val#.decode("utf8")
      address = val.address
      try:
        m = self.dbc.addr_to_msg[address]
      except IndexError:
        raise KeyError(address)
      msgname = m.name#.decode("utf-8")

      # separate definition/value pairs
      def_val = def_val.split()
      values = [int(v) for v in def_val[::2]]
      defs = def_val[1::2]

      # two ways to lookup: address or msg name
      dv[address][sgname] = dict(zip(values, defs))
      dv[msgname][sgname] = dv[address][sgname]

    self.dv = dict(dv)
