import serial
ser = serial.Serial('/dev/tty.usbserial-A900XHJA', 9600)

ser.write('M1F255M2F255D1000M1B255M2B255D1000M1RM2R')

raw_input()

