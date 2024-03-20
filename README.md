# quadruped-robot-ros2

## **Requirements**

- Python 3.10
- [ROS2 humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### RaspberryPI environment setup
```
pip3 install setuptools==58.2.0 
```
```
pip3 install smbus2
```
```
$ mkdir puppy_ws && cd puppy_ws
```
```
$ git clone https://github.com/BruceLin90620/ros2-quadruped-robot.git
```
```
$ git checkout dev
```
- build: 
```
$ colcon build --symlink-install
```
```
$ pip3 install setuptools==58.2.0
```
```
$ sudo apt install ros-humble-joy && sudo apt install ros-humble-teleop-twist-joy
``` 

- Install [Pigpio](https://abyz.me.uk/rpi/pigpio/download.html)
```
wget https://github.com/joan2937/pigpio/archive/master.zip
```
```
unzip master.zip
```
```
cd pigpio-master && make
```
```
sudo make install
```
- Install joy && teleop-twist-joy
```
sudo apt install ros-humble-joy
```
```
sudo apt install ros-humble-teleop-twist-joy
```
### Test
```
sudo pigpiod
```
```
ros2 launch puppy_bringup puppy_bringup.launch.py
```
```
sudo killall pigpiod
```
### Bug Fix
- PermissionError: [Errno 13] Permission denied: '/dev/i2c-1'
```
sudo chmod 777 /dev/i2c-1
```
