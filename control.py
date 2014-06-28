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
        self.ser = serial.Serial('/dev/ttyUSB0', 230400, timeout=1000, xonxoff=0, rtscts=0)
    
    def stop(self):
        self.send_msg(0, 0, 0, 0, 0, 0., 0., 0., 0., 0., 0.)
    
    def send_msg(self, enable, comm_offset_plus, comm_offset_minus, power, control, amplitude, offset, omega, p_gain, d_gain, ff_gain):
        data = struct.pack("<BhhhBiiiiii", enable, comm_offset_plus, comm_offset_minus, power, control,
                                           amplitude * (2**16), offset * (2**16), omega * (2**16),
                                           p_gain * (2**16), d_gain * (2**16), ff_gain * (2**16))
        self.send_raw_msg(data)
    
    def send_raw_msg(self, msg):
        enc_msg = bytearray()
        enc_msg.append(self.MSG_START)
        for x in msg:
            enc_msg.append(0x30 | ((ord(x) & 0xF0) >> 4))
            enc_msg.append(0x30 | (ord(x) & 0x0F))
        enc_msg.append(self.MSG_END)
        
        self.ser.write(enc_msg)

class SerialReader(threading.Thread):
    def __init__(self, ser):
        threading.Thread.__init__(self)
        self.daemon = True
        self.ser = ser
    
    def run(self):
        outfile = open("softscope.fifo", "w")
        
        titles = [ "vel", "des_vel", "pos", "des_pos" ]
        ss_header = struct.pack("=ii", 0x7FD85250, 1)
        for t in titles:
            ss_header += struct.pack("=B", len(t))
            for c in t:
                ss_header += struct.pack("=c", c)
        
        outfile.write(ss_header)
        outfile.flush()
        
        serdecode.init(self.ser)
        serdecode.resync()
        
        while True:
            try:
                while True:
                    s = serdecode.read_frame()
                    
                    (vel, des_vel, pos, des_pos, lower_axis, lower_ring, center_ring) = struct.unpack("=hhhhhhh", s)
                    outfile.write(struct.pack("=ffff", vel/256, des_vel/256, pos/256, des_pos/256))
                    # outfile.write(struct.pack("=ffff", pos, lower_axis, lower_ring, center_ring))
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
        
        self.buttons = Tkinter.Frame(self.master)
        self.buttons.grid(column=0, columnspan=2, row=0)
        
        self.pwr_zero_btn = Tkinter.Button(self.buttons, text="pwr = 0", command=self.pwr_zero_cb)
        self.pwr_zero_btn.pack(side=Tkinter.LEFT)
        
        self.stop_btn = Tkinter.Button(self.buttons, text="   STOP   ", activebackground="red", background="red",
                                       command=self.stop_cb)
        self.stop_btn.pack(side=Tkinter.RIGHT)
        
        self.enable_var = Tkinter.IntVar()
        self.enable_var.set(0)
        Tkinter.Label(self.master, text="Enable driver").grid(column=0, row=1)
        enable_widget = Tkinter.Checkbutton(self.master, variable=self.enable_var, command=self.value_changed_cb)
        enable_widget.grid(column=1, row=1, sticky=Tkinter.W)
        
        self.comm_offset_plus_var = Tkinter.IntVar()
        self.comm_offset_plus_var.set(85)
        Tkinter.Label(self.master, text="comm_offset_plus").grid(column=0, row=2)
        comm_offset_plus_widget = Tkinter.Scale(self.master, variable=self.comm_offset_plus_var, command=lambda x: self.value_changed_cb(),
                                   from_=0, to=6*98, orient=Tkinter.HORIZONTAL)
        comm_offset_plus_widget.grid(column=1, row=2, sticky=Tkinter.W+Tkinter.E)
        
        self.comm_offset_minus_var = Tkinter.IntVar()
        self.comm_offset_minus_var.set(328)
        Tkinter.Label(self.master, text="comm_offset_minus").grid(column=0, row=3)
        comm_offset_minus_widget = Tkinter.Scale(self.master, variable=self.comm_offset_minus_var, command=lambda x: self.value_changed_cb(),
                                   from_=0, to=6*98, orient=Tkinter.HORIZONTAL)
        comm_offset_minus_widget.grid(column=1, row=3, sticky=Tkinter.W+Tkinter.E)
        
        self.power_var = Tkinter.IntVar()
        self.power_var.set(0)
        Tkinter.Label(self.master, text="power").grid(column=0, row=4)
        power_widget = Tkinter.Scale(self.master, variable=self.power_var, command=lambda x: self.value_changed_cb(),
                                     from_=-1000, to=1000, orient=Tkinter.HORIZONTAL)
        power_widget.grid(column=1, row=4, sticky=Tkinter.W+Tkinter.E)
        
        self.control_var = Tkinter.IntVar()
        self.control_var.set(0)
        Tkinter.Label(self.master, text="PD controller").grid(column=0, row=5)
        enable_widget = Tkinter.Checkbutton(self.master, variable=self.control_var, command=self.value_changed_cb)
        enable_widget.grid(column=1, row=5, sticky=Tkinter.W)
        
        self.amplitude_var = Tkinter.DoubleVar()
        self.amplitude_var.set(0)
        Tkinter.Label(self.master, text="amplitude").grid(column=0, row=6)
        amplitude_widget = Tkinter.Scale(self.master, variable=self.amplitude_var, command=lambda x: self.value_changed_cb(),
                                     resolution=0.1, from_=0, to=10, orient=Tkinter.HORIZONTAL)
        amplitude_widget.grid(column=1, row=6, sticky=Tkinter.W+Tkinter.E)
        
        self.offset_var = Tkinter.DoubleVar()
        self.offset_var.set(0)
        Tkinter.Label(self.master, text="offset").grid(column=0, row=7)
        offset_widget = Tkinter.Scale(self.master, variable=self.offset_var, command=lambda x: self.value_changed_cb(),
                                     resolution=0.01, from_=-10., to=10, orient=Tkinter.HORIZONTAL)
        offset_widget.grid(column=1, row=7, sticky=Tkinter.W+Tkinter.E)
        
        self.omega_var = Tkinter.DoubleVar()
        self.omega_var.set(0.01)
        Tkinter.Label(self.master, text="omega").grid(column=0, row=8)
        p_gain_widget = Tkinter.Scale(self.master, variable=self.omega_var, command=lambda x: self.value_changed_cb(),
                                     resolution=0.01, from_=0.05, to=20.0, orient=Tkinter.HORIZONTAL)
        p_gain_widget.grid(column=1, row=8, sticky=Tkinter.W+Tkinter.E)
        
        self.p_gain_var = Tkinter.DoubleVar()
        self.p_gain_var.set(1.)
        Tkinter.Label(self.master, text="P gain").grid(column=0, row=9)
        p_gain_widget = Tkinter.Scale(self.master, variable=self.p_gain_var, command=lambda x: self.value_changed_cb(),
                                     resolution=0.01, from_=0., to=100., orient=Tkinter.HORIZONTAL)
        p_gain_widget.grid(column=1, row=9, sticky=Tkinter.W+Tkinter.E)
        
        self.d_gain_var = Tkinter.DoubleVar()
        self.d_gain_var.set(0.1)
        Tkinter.Label(self.master, text="D gain").grid(column=0, row=10)
        d_gain_widget = Tkinter.Scale(self.master, variable=self.d_gain_var, command=lambda x: self.value_changed_cb(),
                                     resolution=0.01, from_=-1., to=1., orient=Tkinter.HORIZONTAL)
        d_gain_widget.grid(column=1, row=10, sticky=Tkinter.W+Tkinter.E)
        
        self.ff_gain_var = Tkinter.DoubleVar()
        self.ff_gain_var.set(0.)
        Tkinter.Label(self.master, text="FF gain").grid(column=0, row=11)
        ff_gain_widget = Tkinter.Scale(self.master, variable=self.ff_gain_var, command=lambda x: self.value_changed_cb(),
                                     resolution=0.01, from_=-50., to=50., orient=Tkinter.HORIZONTAL)
        ff_gain_widget.grid(column=1, row=11, sticky=Tkinter.W+Tkinter.E)
        
        # self.comm_value = Tkinter.IntVar()
        # self.comm_value.set(0)
        # comm_widget = Tkinter.Scale(self.master, variable=self.comm_value, command=self.value_changed_cb,
        #                             from_=-10000, to=10000, orient=Tkinter.HORIZONTAL)
        # comm_widget.grid(fill=Tkinter.X, expand=1)
    
    def stop_cb(self):
        self.enable_var.set(0)
        self.value_changed_cb()
    
    def pwr_zero_cb(self):
        self.power_var.set(0)
        self.amplitude_var.set(0)
        self.value_changed_cb()
    
    def value_changed_cb(self):
        self.dev.send_msg(self.enable_var.get(), self.comm_offset_plus_var.get(), self.comm_offset_minus_var.get(),
                          self.power_var.get(), self.control_var.get(), self.amplitude_var.get(), self.offset_var.get(),
                          self.omega_var.get(), self.p_gain_var.get(), self.d_gain_var.get(), self.ff_gain_var.get())

dev = Device()
atexit.register(lambda: dev.stop())

SerialReader(dev.ser).start()

root = Tkinter.Tk()
root.title("BLDC motor control")
gui = GUI(root, dev)

root.mainloop()
