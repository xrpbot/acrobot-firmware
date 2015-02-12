##
## Derived from examples in the libopencm3 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##
## Modified by Norbert Braun for the XRPBot project.

BINARY = brushless
OBJS += usart.o serencode.o statusled.o ssi.o sensors.o trajectory.o pwm.o quadrature.o adc.o transport.o current.o util.o vel_ctrl.o pumping_ctrl.o

LDSCRIPT = stm32f4-discovery.ld

include Makefile.include

