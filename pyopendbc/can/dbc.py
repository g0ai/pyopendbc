#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <vector>
#include <mutex>
#include <iterator>
#include <cstring>
#include <clocale>

#include "opendbc/can/common.h"
#include "opendbc/can/common_dbc.h"

import re
import os

from .common import honda_checksum, hkg_can_fd_checksum
from .common_dbc import DBC, ChecksumState, SignalType, Msg, Signal, Val

from opendbc import DBC_PATH

bo_regexp = r"(^BO_ (\w+) (\w+) *: (\w+) (\w+))"
sg_regexp = r"(^SG_ (\w+) : (\d+)\|(\d+)@(\d+)([\+|\-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\) \[([0-9.+\-eE]+)\|([0-9.+\-eE]+)\] \"(.*)\" (.*))"
sgm_regexp = r"(^SG_ (\w+) (\w+) *: (\d+)\|(\d+)@(\d+)([\+|\-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\) \[([0-9.+\-eE]+)\|([0-9.+\-eE]+)\] \"(.*)\" (.*))"
val_regexp = r"(VAL_ (\w+) (\w+) (\s*[-+]?[0-9]+\s+\".+?\"[^;]*))"
val_split_regexp = r"([\"]+)"  # split on "

# def DBC_ASSERT(condition, message):
#   # do {                                                             
#   if (not condition):                                          
#     std::stringstream is;                                        
#     is << "[" << dbc_name << ":" << line_num << "] " << message; 
#     throw std::runtime_error(is.str());                                                                                        
#   # } while (false)

def DBC_ASSERT(dbc_name, line_num, condition, message):
    if (not condition):
        raise Exception(f"[{dbc_name}:{line_num}] {message}")

def startswith(string, prefix):
  return string.find(prefix) == 0

# TODO: overload
def startswithlist(string, prefix_list):
  for prefix in prefix_list:
    if (startswith(string, prefix)): return True
  return False

def endswith(string, suffix):
  return str.find(suffix) == (len(string) - len(suffix))

# TODO
def trim(s, t = " \t\n\r\f\v"):
  # s.erase(s.find_last_not_of(t) + 1)
  # return s.erase(0, s.find_first_not_of(t))
  return s.strip(t)

def get_checksum(dbc_name):
  s = ChecksumState()
  if (startswithlist(dbc_name, ["honda_", "acura_"])):
    s = ChecksumState(4, 2, 3, 5, False, SignalType.HONDA_CHECKSUM, honda_checksum)
  # elif (startswithlist(dbc_name, ["toyota_", "lexus_"])):
  #   s = ChecksumState(8, -1, 7, -1, False, SignalType.TOYOTA_CHECKSUM, toyota_checksum)
  elif (startswith(dbc_name, "hyundai_canfd")):
    s = ChecksumState(16, -1, 0, -1, True, SignalType.HKG_CAN_FD_CHECKSUM, hkg_can_fd_checksum)
  # elif (startswith(dbc_name, "vw_mqb_2010")):
  #   s = ChecksumState(8, 4, 0, 0, True, SignalType.VOLKSWAGEN_MQB_CHECKSUM, volkswagen_mqb_checksum)
  # elif (startswith(dbc_name, "vw_golf_mk4")):
  #   s = ChecksumState(8, 4, 0, -1, True, SignalType.XOR_CHECKSUM, xor_checksum)
  # elif (startswith(dbc_name, "subaru_global_")):
  #   s = ChecksumState(8, -1, 0, -1, True, SignalType.SUBARU_CHECKSUM, subaru_checksum)
  # elif (startswith(dbc_name, "chrysler_")):
  #   s = ChecksumState(8, -1, 7, -1, False, SignalType.CHRYSLER_CHECKSUM, chrysler_checksum)
  # elif (startswith(dbc_name, "comma_body")):
  #   s = ChecksumState(8, 4, 7, 3, False, SignalType.PEDAL_CHECKSUM, pedal_checksum)
  return s


# Signal& s, ChecksumState* chk, const std::string& dbc_name, int line_num
def set_signal_type(s, chk, dbc_name, line_num):
  s.calc_checksum = None
  if chk:
    if s.name == "CHECKSUM":
      DBC_ASSERT(dbc_name, line_num, chk.checksum_size == -1 or s.size == chk.checksum_size, f"CHECKSUM is not {chk.checksum_size} bits long")
      DBC_ASSERT(dbc_name, line_num, chk.checksum_start_bit == -1 or (s.start_bit % 8) == chk.checksum_start_bit, " CHECKSUM starts at wrong bit")
      DBC_ASSERT(dbc_name, line_num, s.is_little_endian == chk.little_endian, "CHECKSUM has wrong endianness")
      DBC_ASSERT(dbc_name, line_num, chk.calc_checksum != None, "CHECKSUM calculate function not supplied")
      s.type = chk.checksum_type
      s.calc_checksum = chk.calc_checksum
    elif s.name == "COUNTER":
      DBC_ASSERT(dbc_name, line_num, chk.counter_size == -1 or s.size == chk.counter_size, f"COUNTER is not {chk.counter_size} bits long")
      DBC_ASSERT(dbc_name, line_num, chk.counter_start_bit == -1 or (s.start_bit % 8) == chk.counter_start_bit, "COUNTER starts at wrong bit")
      DBC_ASSERT(dbc_name, line_num, chk.little_endian == s.is_little_endian, "COUNTER has wrong endianness")
      s.type = SignalType.COUNTER


# const std::string &dbc_name, std::istream &stream, ChecksumState *checksum, bool allow_duplicate_msg_name
def dbc_parse_from_stream(dbc_name, file, checksum, allow_duplicate_msg_name=True):
  address = 0
  address_set = []
  msg_name_set = []
  signal_name_sets = {}
  signals = {}
  dbc = DBC()
  dbc.name = dbc_name
  # std::setlocale(LC_NUMERIC, "C");

  # used to find big endian LSB from MSB and size
  be_bits = []
  for i in range(64):
    for j in range(7, -1, -1): # (int j = 7; j >= 0; j--) {
      be_bits.append(j + i * 8)

  line = ""
  line_num = 0
  # std::smatch match;
  # TODO: see if we can speed up the regex statements in this loop, SG_ is specifically the slowest
  with open(file, "r") as f:
    for line in f:
      line = trim(line)
      line_num += 1
      if startswith(line, "BO_ "):
        # new group
        match = re.match(bo_regexp, line)
        DBC_ASSERT(dbc_name, line_num, bool(match), "bad BO: " + line)

        msg = Msg() # dbc.msgs.append()
        address = msg.address = int(match.group(2))  # could be hex
        msg.name = str(match.group(3))
        msg.size = int(match.group(4))

        dbc.msgs.append(msg)

        # check for duplicates
        DBC_ASSERT(dbc_name, line_num, address not in address_set, f"Duplicate message address: {address} ({msg.name})")
        address_set.append(address)

        if not allow_duplicate_msg_name:
          DBC_ASSERT(dbc_name, line_num, msg.name not in msg_name_set, f"Duplicate message name: {msg.name}")
          msg_name_set.insert(msg.name)
        
      elif startswith(line, "SG_ "):
        # new signal
        offset = 0
        match = re.match(sg_regexp, line)
        if not bool(match):
          match = re.match(sgm_regexp, line)
          DBC_ASSERT(dbc_name, line_num, bool(match), f"bad SG: {line}")
          offset = 1

        sig = Signal()
        sig.name = match.group(2)
        sig.start_bit = int(match.group(offset + 3))
        sig.size = int(match.group(offset + 4))
        sig.is_little_endian = int(match.group(offset + 5)) == 1
        sig.is_signed = str(match.group(offset + 6)) == "-"
        sig.factor = float(match.group(offset + 7))
        sig.offset = float(match.group(offset + 8))
        set_signal_type(sig, checksum, dbc_name, line_num)
        if (sig.is_little_endian):
          sig.lsb = sig.start_bit
          sig.msb = sig.start_bit + sig.size - 1
        else:
          it = be_bits.index(sig.start_bit)
          # sig.lsb = be_bits[(it - be_bits.begin()) + sig.size - 1]
          sig.lsb = be_bits[it + sig.size - 1]
          sig.msb = sig.start_bit
        
        DBC_ASSERT(dbc_name, line_num, sig.lsb < (64 * 8) and sig.msb < (64 * 8), f"Signal out of bounds: {line}")

        # Check for duplicate signal names
        # print(signal_name_sets, address, sig.name)
        DBC_ASSERT(dbc_name, line_num, sig.name not in signal_name_sets.items(), f"Duplicate signal name: {sig.name}")
        if address in signal_name_sets:
          signal_name_sets[address].append(sig.name)
          signals[address].append(sig)
        else:
          signal_name_sets[address] = [sig.name]
          signals[address] = [sig]

      elif (startswith(line, "VAL_ ")):
        # new signal value/definition
        match = re.match(val_regexp, line)
        DBC_ASSERT(dbc_name, line_num, bool(match), "bad VAL: " + line)

        val = Val()
        val.address = int(match.group(2))  # could be hex
        val.name = match.group(3)

        defvals = match.group(4)
        s = ' '.join([trim(w).upper().replace(" ", "_") for w in defvals.split("\"")])
        val.def_val = s
        val.def_val = trim(val.def_val)
        dbc.vals.append(val)

  for m in dbc.msgs:
    m.sigs = signals[m.address]
    dbc.addr_to_msg[m.address] = m
    dbc.name_to_msg[m.name] = m
  
  for v in dbc.vals:
    v.sigs = signals[v.address]
  
  return dbc


def dbc_parse(dbc_path):
  dbc_name = os.path.basename(dbc_path)

  checksum = get_checksum(dbc_name)
  return dbc_parse_from_stream(dbc_name, dbc_path, checksum.get())


def get_dbc_root_path():
  basedir = os.getenv("BASEDIR")
  if basedir is not None:
    return str(basedir) # + "/opendbc")
  else:
    return DBC_PATH


def dbc_lookup(dbc_name):
  dbc_parent = get_dbc_root_path()
  dbc_path = os.path.join(dbc_parent, f'{dbc_name}.dbc')
  dbc = dbc_parse(dbc_path)
  dbc.name = dbc_name
  return dbc


def get_dbc_names():
  dbc_parent = get_dbc_root_path()
  dbcs = []
  for file in os.listdir(dbc_parent):
    if not file.startswith("_") and file.endswith(".dbc"):
        dbcs.append(os.path.join(dbc_parent, file))

  return dbcs
