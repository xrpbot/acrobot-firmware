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

class Device:
    MSG_START = 0x40
    MSG_END = 0x41
    
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB0', 2500000, timeout=1000, xonxoff=0, rtscts=0)
    
    def stop(self):
        self.send_msg(0, 0)
    
    def send_msg(self, mag, phase):
        data = struct.pack("<ii", mag, phase*1024)
        self.send_raw_msg(data)
    
    def send_raw_msg(self, msg):
        enc_msg = bytearray()
        enc_msg.append(self.MSG_START)
        for x in msg:
            enc_msg.append(0x30 | ((ord(x) & 0xF0) >> 4))
            enc_msg.append(0x30 | (ord(x) & 0x0F))
        enc_msg.append(self.MSG_END)
        
        self.ser.write(enc_msg)

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
        outfile.write(ss_header([ "I1", "I2", "phi", "cnt" ]))
        outfile.flush()
        
        serdecode.init(self.ser)
        serdecode.resync()
        
        while True:
            try:
                while True:
                    s = serdecode.read_frame()
                    
                    (cur1, cur2, phi, cnt) = struct.unpack("=hhhh", s)
                    outfile.write(struct.pack("=ffff", cur1-4096., cur2-4096., phi, cnt))
                    outfile.flush()
            except serdecode.SerDecodeError as error:
                print(error)
                serdecode.resync()
        
        outfile.close()

class GUI:
    def __init__(self, master, dev):
        self.master = master
        self.dev = dev
        
        self.master.grid_columnconfigure(0, weight=0)
        self.master.grid_columnconfigure(1, weight=1)
        
        self.mag_var = Tkinter.DoubleVar()
        self.mag_var.set(0)
        Tkinter.Label(self.master, text="mag").grid(column=0, row=0)
        comm_offset_plus_widget = Tkinter.Scale(self.master, variable=self.mag_var, command=lambda x: self.value_changed_cb(),
                                   from_=0, to=1024, orient=Tkinter.HORIZONTAL)
        comm_offset_plus_widget.grid(column=1, row=0, sticky=Tkinter.W+Tkinter.E)
        
        self.phase_var = Tkinter.DoubleVar()
        self.phase_var.set(0)
        Tkinter.Label(self.master, text="phase").grid(column=0, row=1)
        comm_offset_minus_widget = Tkinter.Scale(self.master, variable=self.phase_var, command=lambda x: self.value_changed_cb(),
                                   from_=-10., to=10., resolution=0.01, orient=Tkinter.HORIZONTAL)
        comm_offset_minus_widget.grid(column=1, row=1, sticky=Tkinter.W+Tkinter.E)
    
    def value_changed_cb(self):
        self.dev.send_msg(self.mag_var.get(), self.phase_var.get())

dev = Device()
atexit.register(lambda: dev.stop())

SerialReader(dev.ser).start()

root = Tkinter.Tk()
root.title("BLDC motor control")
gui = GUI(root, dev)

root.mainloop()
