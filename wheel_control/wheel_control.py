import serial
import sys
import types
import curses

if len(sys.argv) == 1:
    print "Usage: python wheel-control.py /dev/ttyUSB0"
    exit(-1)

ser = None

left_wheel_speed = 0
right_wheel_speed = 0

win = None

def set_wheel_speeds(new_left_wheel_speed, new_right_wheel_speed):
    global win, ser, left_wheel_speed, right_wheel_speed
    if type(new_left_wheel_speed) != types.IntType or type(new_right_wheel_speed) != types.IntType \
        or new_left_wheel_speed < -127 or new_left_wheel_speed > 127 \
        or new_right_wheel_speed < -127 or new_right_wheel_speed > 127:
        win.addstr("\nError!")
        kill()
        exit(-1)
    if new_left_wheel_speed == left_wheel_speed and new_right_wheel_speed == right_wheel_speed:
        return
    cmd = "1 %d 2 %d" % (new_left_wheel_speed, new_right_wheel_speed)
    byte_count = len(cmd)
    while ser.write(cmd) != byte_count:
        win.addstr("\nError!")
        pass
    left_wheel_speed = new_left_wheel_speed
    right_wheel_speed = new_right_wheel_speed

def kill():
    global current_action
    current_action = "Killing motors"
    set_wheel_speeds(0, 0)

def accelerate(left_goal_speed, right_goal_speed):
    global left_wheel_speed, right_wheel_speed, current_action
    current_action = "Accelerating left motor to %d, right motor to %d" % (left_goal_speed, right_goal_speed)
    # todo: gentle acceleration
    set_wheel_speeds(left_goal_speed, right_goal_speed)

def stop():
    global left_wheel_speed, right_wheel_speed, current_action
    current_action = "Stopping"
    # exponential decelaration?
    kill()

def forward(speed):
    global current_action
    current_action = "Accelerating forward to speed %d" % speed
    accelerate(speed, speed)

def turn(angle_degrees):
    global current_action
    current_action = "Turning %d degrees" % angle_degrees
    if angle_degrees > 0:
        accelerate(0, 20)
    else:
        accelerate(20, 0)

def status_display():
    global win, left_wheel_speed, right_wheel_speed, current_action
    win.move(0, 0)
    win.clrtoeol()
    win.move(1, 0)
    win.clrtoeol()
    win.addstr(0, 0, "L: %d, R: %d\n" % (left_wheel_speed, right_wheel_speed))
    win.addstr(1, 0, current_action)

def main(w):
    global win, ser
    ser = serial.Serial(sys.argv[1], 9600)
    win = w
    win.clear()
    # start stopped
    kill()
    win.timeout(1000)
    key=""
    while True:
        status_display()
        try:
            key = win.getkey()
            if key == "q":
                kill()
                break 
            elif key == "KEY_UP":
                forward(20)
            elif key == "KEY_DOWN":
                forward(-20)
            elif key == "KEY_LEFT":
                turn(-20)
            elif key == "KEY_RIGHT":
                turn(20)
        except Exception as e:
            # No input
            stop()

curses.wrapper(main)


