#!/usr/bin/python
import serial
import struct
import time
import binascii

ser = serial.Serial('/dev/ttyUSB0', 2500000, timeout=0, xonxoff=0, rtscts=0)

MSG_START = 0xF1
MSG_ESCAPE = 0xF2
MSG_END = 0xF3

def frame_msg(msg):
    raw_msg = struct.pack("=B", MSG_START)
    for c in msg:
        if ord(c) & 0x80:
            raw_msg += struct.pack("=BB", MSG_ESCAPE, ord(c) ^ 0x80)
        else:
            raw_msg += c
    raw_msg += struct.pack("=B", MSG_END)
    return raw_msg

def send_msg(dev, msg):
    msg += struct.pack("<I", binascii.crc32(msg) & 0xFFFFFFFF)
    dev.write(frame_msg(msg))

send_msg(ser, struct.pack("=B", 0x42))
ser.read()
