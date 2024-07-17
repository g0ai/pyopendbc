#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

from enum import Enum
import os

# from .dbc import dbc_parse_from_stream, dbc_lookup # , get_dbc_names

class SignalPackValue:
  name = ""
  value = 0.0

class SignalValue:
  address = 0
  ts_nanos = 0
  name = ""
  value = 0.0  # latest value
  all_values = []  # all values from this cycle

class SignalType(Enum):
  DEFAULT = 0
  COUNTER = 1
  HONDA_CHECKSUM = 2
  TOYOTA_CHECKSUM = 3
  PEDAL_CHECKSUM = 4
  VOLKSWAGEN_MQB_CHECKSUM = 5
  XOR_CHECKSUM = 6
  SUBARU_CHECKSUM = 7
  CHRYSLER_CHECKSUM = 8
  HKG_CAN_FD_CHECKSUM = 9


class Signal:
  name = ""
  start_bit = 0
  ms = 0
  lsb = 0
  size = 0
  is_signed = False
  factor = 0.0
  offset = 0.0
  is_little_endian = False
  type = SignalType.DEFAULT
  
  # uint32_t address, const Signal &sig, const std::vector<uint8_t> &d
  def calc_checksum(address, sig, d):
    return 0

class Msg:
  name = ""
  address = 0
  size = 0
  sigs = []


class Val:
  name = ""
  address = 0
  def_val = ""
  sigs = []


class DBC:
  name = ""
  msgs = []
  vals = []
  addr_to_msg = {} # (0.0, Msg())
  name_to_msg = {} # ("", Msg())


class ChecksumState:
  # checksum_size = 0
  # counter_size = 0
  # checksum_start_bit = 0
  # counter_start_bit = 0
  # little_endian = False
  # checksum_type = SignalType.DEFAULT
  
  def __init__(self,
               checksum_size=0,
               counter_size=0,
               checksum_start_bit=0,
               counter_start_bit=0,
               little_endian=False,
               checksum_type = SignalType.DEFAULT,
               calc_checksum=None):
    self.checksum_size = checksum_size
    self.counter_size = counter_size
    self.checksum_start_bit = checksum_start_bit
    self.counter_start_bit = counter_start_bit
    self.little_endian = little_endian
    self.checksum_type = checksum_type
    self.calc_checksum = calc_checksum
  
  def get(self):
    return self

