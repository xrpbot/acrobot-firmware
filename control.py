#!/usr/bin/python
from __future__ import print_function
from __future__ import division
import sys
import serdecode
import threading
import serial
import struct
import Tkinter
import atexit
import binascii

class Device:
    MSG_START = 0xF1
    MSG_ESCAPE = 0xF2
    MSG_END = 0xF3
    
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB0', 2500000, timeout=1000, xonxoff=0, rtscts=0)
    
    def stop(self):
        self.send_msg(0, 0, 0)
    
    def _frame_msg(self, msg):
        raw_msg = struct.pack("=B", self.MSG_START)
        for c in msg:
            if ord(c) & 0x80:
                raw_msg += struct.pack("=BB", self.MSG_ESCAPE, ord(c) ^ 0x80)
            else:
                raw_msg += c
        raw_msg += struct.pack("=B", self.MSG_END)
        return raw_msg

    def send_raw_msg(self, data):
        data += struct.pack("<I", binascii.crc32(data) & 0xFFFFFFFF)
        self.ser.write(self._frame_msg(data))
    
    def send_msg(self, gain, offset, scale):
        data = struct.pack("<Biii", 0x01, gain, offset, scale)
        self.send_raw_msg(data)


def ss_header(titles):
    header = struct.pack("=ii", 0x7FD85250, 1)
    for t in titles:
        header += struct.pack("=B", len(t))
        for c in t:
            header += struct.pack("=c", c)
    return header

class SerialReader(threading.Thread):
    def __init__(self, ser):
        threading.Thread.__init__(self)
        self.daemon = True
        self.ser = ser
    
    def run(self):
        outfile = open("softscope.fifo", "w")
        outfile.write(ss_header([ "pos_active", "pos_passive", "vel_active", "vel_passive" ]))
        outfile.flush()
        
        serdecode.init(self.ser)
        serdecode.resync()
        
        while True:
            try:
                while True:
                    s = serdecode.read_frame()
                    values = struct.unpack("=ffff", s)
                    outfile.write(struct.pack("=ffff", *values))
                    outfile.flush()
            except serdecode.SerDecodeError as error:
                print(error)
                serdecode.resync()
        
        outfile.close()

class GUI:
    def __init__(self, master, dev):
        self.master = master
        self.dev = dev
        
        params = [ [ "gain", 0, -500, 500 ],
                   [ "offset", 0, -100, 100 ],
                   [ "scale", 10, 0, 50] ]
        
        self.master.grid_columnconfigure(0, weight=0)
        self.master.grid_columnconfigure(1, weight=1)
        
        self.vars = []
        for (p, pos) in zip(params, range(0, len(params))):
            (txt, def_value, min_value, max_value) = p
            var = Tkinter.DoubleVar()
            var.set(def_value)
            Tkinter.Label(self.master, text=txt).grid(column=0, row=pos)
            widget = Tkinter.Scale(self.master, variable=var, command=lambda x: self.value_changed_cb(),
                                   from_=min_value, to=max_value, orient=Tkinter.HORIZONTAL)
            widget.grid(column=1, row=pos, sticky=Tkinter.W+Tkinter.E)
            self.vars.append(var)
    
    def value_changed_cb(self):
        self.dev.send_msg(*([v.get() for v in self.vars]))

dev = Device()
atexit.register(lambda: dev.stop())

SerialReader(dev.ser).start()

root = Tkinter.Tk()
root.title("BLDC motor control")
gui = GUI(root, dev)

root.mainloop()
