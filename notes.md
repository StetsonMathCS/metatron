

# Start all services

```
roscore
```

# Cameras

Start services

```
roslaunch usb_cam usb_cams.launch
```

## View camera images

```
rosrun rqt_image_view rqt_image_view
```

That will open a window where you can select the image to view.

Another option:

```
rosrun image_view image_view image:=/usb_cam/left/image_raw
```


## Camera calibration

```
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.246 right:=/usb_cam/right/image_raw left:=/usb_cam/left/image_raw right_camera:=/usb_cam/right left_camera:=/usb_cam/left --approximate=0.1
```

## Build catkin packages (all of them)

```
cd ~/catkin
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python2 -DPYTHON_INCLUDE_DIR=/usr/include/python2.7 -DPYTHON_LIBRARY=/usr/lib/libpython2.7.so install
```

# LIDAR

View points:

```
roslaunch rplidar_ros view_rplidar.launch
```

See raw 2D coordinates:

```
roslaunch rplidar_ros rplidar.launch
rosrun rplidar_ros rplidarNodeClient
```


# Wheel control


## Change code on Arduino Nano

```
~/arduino-1.6.9/arduino-cli --upload --board arduino:avr:nano:cpu=atmega168 --port /dev/ttyUSB0 -v wheel_control.ino
```

Wheels:

- 1 = front right
- 2 = front left
- 3 = rear right
- 4 = rear left

# Arm control

Arm: see diagram


# Wiring

## Arduino Nano to Sabertooth 2x12

Sabertooth orange -> GND (near VIN)
Sabertooth yellow -> TX1

## Arduino Nano to HC-SR04 sonar sensor

HC-SR04 Vcc  -> +5V
HC-SR04 Trig -> D3
HC-SR04 Echo -> D2
HC-SR04 Gnd  -> GND (near VIN)

