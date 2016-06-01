import serial
ser = serial.Serial('/dev/ttyUSB0', 9600)

# send two numbers to serial port:
# first = motor (1-4)
# second = speed (-127 - 127)

# ser.write('1 20')


