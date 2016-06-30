
import serial
import time

class Arm:

    def __init__(self):
        self.gripperPosition = 0
        self.gripperMinPosition = 1
        self.gripperMaxPosition = 10
        self.gripperMotor = 4
        self.gripperOpen = "F"
        self.gripperClose = "B"
        self.gripperOpenDelay = 125
        self.gripperCloseDelay = 125

        self.elbowPosition = 0
        self.elbowMinPosition = 60
        self.elbowMaxPosition = 230
        self.elbowMotor = 1
        self.elbowUp = "B"
        self.elbowDown = "F"
        self.elbowUpDelay = 43
        self.elbowDownDelay = 40

        self.wristPosition = 0
        self.wristMinPosition = 130
        self.wristMaxPosition = 220
        self.wristMotor = 2
        self.wristUp = "F"
        self.wristDown = "B"
        self.wristUpDelay = 47
        self.wristDownDelay = 45

        self.shoulderPosition = 0
        self.shoulderMinPosition = 90
        self.shoulderMaxPosition = 190
        self.shoulderMotor = 3
        self.shoulderUp = "F"
        self.shoulderDown = "B"
        self.shoulderUpDelay = 60
        self.shoulderDownDelay = 40

        self.com = serial.Serial("/dev/ttyUSB1", 9600)

        # reset all motors, just to be safe
        self.sendMessage("M1RM2RM3RM4R")

    def calibrate(self):
        # set gripper to wide-open
        while True:
            answer = raw_input("Is gripper wide open? (y/n): ")
            if answer == "y":
                self.gripperPosition = 10
                break
            elif answer == "n":
                self.activateMotor(self.gripperMotor, self.gripperOpen, 250)
        # set elbow to horizontal
        while True:
            answer = raw_input("Is elbow straight or up or down? (s/u/d): ")
            if answer == "s":
                self.elbowPosition = 180
                break
            elif answer == "u":
                self.activateMotor(self.elbowMotor, self.elbowDown, 250)
            elif answer == "d":
                self.activateMotor(self.elbowMotor, self.elbowUp, 250)
        # set shoulder to horizontal
        while True:
            answer = raw_input("Is shoulder straight or up or down? (s/u/d): ")
            if answer == "s":
                self.shoulderPosition = 180
                break
            elif answer == "u":
                self.activateMotor(self.shoulderMotor, self.shoulderDown, 250)
            elif answer == "d":
                self.activateMotor(self.shoulderMotor, self.shoulderUp, 250)
        # set wrist to horizontal
        while True:
            answer = raw_input("Is wrist straight or up or down? (s/u/d): ")
            if answer == "s":
                self.wristPosition = 180
                break
            elif answer == "u":
                self.activateMotor(self.wristMotor, self.wristDown, 250)
            elif answer == "d":
                self.activateMotor(self.wristMotor, self.wristUp, 250)

    def reset(self):
        self.setGripperPosition(10)
        self.setElbowPosition(180)
        self.setShoulderPosition(180)
        self.setWristPosition(180)

    def setGripperPosition(self, pos):
        if pos == self.gripperPosition: return
        if pos >= self.gripperMinPosition and pos <= self.gripperMaxPosition:
            diff = abs(self.gripperPosition - pos)
            if pos > self.gripperPosition:
                direction = self.gripperOpen
                delay = diff * self.gripperOpenDelay
            else:
                direction = self.gripperClose
                delay = diff * self.gripperCloseDelay
            self.activateMotor(self.gripperMotor, direction, delay)
            self.gripperPosition = pos

    def setElbowPosition(self, pos):
        if pos == self.elbowPosition: return
        if pos >= self.elbowMinPosition and pos <= self.elbowMaxPosition:
            diff = abs(self.elbowPosition - pos)
            if pos > self.elbowPosition:
                direction = self.elbowDown
                delay = diff * self.elbowDownDelay
            else:
                direction = self.elbowUp
                delay = diff * self.elbowUpDelay
            self.activateMotor(self.elbowMotor, direction, delay)
            self.elbowPosition = pos

    def setWristPosition(self, pos):
        if pos == self.wristPosition: return
        if pos >= self.wristMinPosition and pos <= self.wristMaxPosition:
            diff = abs(self.wristPosition - pos)
            if pos > self.wristPosition:
                direction = self.wristDown
                delay = diff * self.wristDownDelay
            else:
                direction = self.wristUp
                delay = diff * self.wristUpDelay
            self.activateMotor(self.wristMotor, direction, delay)
            self.wristPosition = pos

    def setShoulderPosition(self, pos):
        if pos == self.shoulderPosition: return
        if pos >= self.shoulderMinPosition and pos <= self.shoulderMaxPosition:
            diff = abs(self.shoulderPosition - pos)
            if pos > self.shoulderPosition:
                direction = self.shoulderDown
                delay = diff * self.shoulderDownDelay
            else:
                direction = self.shoulderUp
                delay = diff * self.shoulderUpDelay
            self.activateMotor(self.shoulderMotor, direction, delay)
            self.shoulderPosition = pos

    def activateMotor(self, motor, direction, duration):
        self.sendMessage("M%d%s255D%dM%dR" % (motor, direction, duration, motor))
        time.sleep(duration / 1000.0)

    def sendMessage(self, msg):
        self.com.write(msg)

    def grab(self):
        self.setWristPosition(130)
        raw_input()
        self.setElbowPosition(190)
        raw_input()
        self.setShoulderPosition(185)
        raw_input()
        self.setElbowPosition(230)
        raw_input()
        self.setGripperPosition(2)

    def lift(self):
        self.setElbowPosition(220)
        self.setShoulderPosition(120)
        self.setWristPosition(180)

    def putDown(self):
        self.setShoulderPosition(135)
        self.setWristPosition(135)
        self.setShoulderPosition(185)
        self.setElbowPosition(220)
        self.setGripperPosition(10)


if __name__ == "__main__":
    arm = Arm()
    arm.calibrate()
    raw_input("grab")
    arm.grab()
    raw_input("lift")
    arm.lift()
    raw_input("put down")
    arm.putDown()
    raw_input("reset")
    arm.reset()
    raw_input("done")


