# Limo ROS2 Humble


## Download and Setup
Star this repository to keep up to date with the latest changes.


### Install ROS and necessary dependencies

If ROS2 is not yet installed, please follow the official installation guide for [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html).

### Setup workspace

Open a terminal (CTRL+ALT+T) and run the following commands:

```bash
mkdir -p ~/limo_ws/src
cd ~/limo_ws/src
git clone https://github.com/idra-lab/limo_ros2.git
cd ..
```
### Run the setup script to install dependencies

```bash
chmod +x src/limo_ros2/setup_limo_ros2.sh
./src/limo_ros2/setup_limo_ros2.sh
```

### Alternatively, manually install the required packages:

Install custom ros2-gz-control package, which provides the necessary interfaces to control robots in Gazebo with ROS2.

```bash
cd src
git clone https://github.com/tamasso-parec/gz_ros2_control.git
cd ..
```

Download and install joint-state-publisher-gui package.This package is used to visualize the joint control.

```bash
sudo apt-get install ros-humble-joint-state-publisher-gui 
```

Download and install rqt-robot-steering plug-in, rqt_robot_steering is a ROS tool closely related to robot motion control, it can send the control command of robot linear motion and steering motion, and the robot motion can be easily controlled through the sliding bar

```bash
sudo apt-get install ros-humble-rqt-robot-steering 
```

Download and install teleop-twist-keyboard

```bash
sudo apt-get install ros-humble-teleop-twist-keyboard
```

### Install additional ros and gz dependencies: 

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```
### Fix Gazebo installation:

```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
```

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```
```bash
apt-get install ros-humble-ros-gzharmonic
```

Source your ROS2 Humble installation:

```bash
source /opt/ros/humble/setup.bash
```
### Install rosdep: 

```bash
sudo apt-get install python-pip
sudo pip install -U rosdep
sudo rosdep init
rosdep update
```

Install dependencies using `rosdep`:
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
### Build the workspace: 

```bash
source /opt/ros/humble/setup.bash
```

```bash
colcon build --symlink-install
```
Source the workspace:

```bash
source install/setup.bash
```

## Verify the installation by launching the simulation:

```bash
ros2 launch limo_car ackermann_gazebo.launch.py
```

In another terminal, source the workspace again:

```bash
source /opt/ros/humble/setup.bash
source ~/limo_ws/install/setup.bash
```

and run the robot steering GUI:
```bash
ros2 run rqt_robot_steering rqt_robot_steering --ros-args --remap /cmd_vel:=/ackermann_steering_controller/reference 
```
Or using the teleop twist keyboard tool to command the robot with keybord inputs: 

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/ackermann_steering_controller/reference -p stamped:=true
```

## Maintainer

Tommaso Faraci  
IDRA Lab, University of Trento  
Email: [tommaso.faraci@unitn.it](mailto:tommaso.faraci@unitn.it)











# mycobot_ros2

## URDF Model Graph

[mycobot 280 m5](./mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5.urdf)

![280 m5](./demo_img/280m5/280_m5.png)

[mycobot 280 m5 pump](./mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5_with_pump.urdf)

![280 m5 pump](./demo_img/280m5/280_m5_pump.png)

[mycobot 280 m5 camera flange](./mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5_with_camera_flange.urdf)

![280 m5 camera flange](./demo_img/280m5/280_m5_camera_flange.png)

[mycobot 280 m5 camera flange & pump](./mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5_with_camera_flange_pump.urdf)

![280 m5 camera flange & pump](./demo_img/280m5/280_m5_camera_flange_pump.png)

[mycobot 280 m5 adaptive gripper](./mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5_adaptive_gripper.urdf)
![280 m5 adaptive gripper](./demo_img/280m5/280_m5_adaptive_gripper.png)

[mycobot 280 pi](./mycobot_description/urdf/mycobot_280_pi/mycobot_280_pi.urdf)

![280 pi](./demo_img/280pi/280_pi.png)

[mycobot 280 pi pump](./mycobot_description/urdf/mycobot_280_pi/mycobot_280_pi_with_pump.urdf)

![280 pi pump](./demo_img/280pi/280_pi_pump.png)

[mycobot 280 pi camera flange](./mycobot_description/urdf/mycobot_280_pi/mycobot_280_pi_with_camera_flange.urdf)

![280 pi camera flange](./demo_img/280pi/280_pi_camera_flange.png)

[mycobot 280 pi camera flange & pump](./mycobot_description/urdf/mycobot_280_pi/mycobot_280_pi_with_camera_flange_pump.urdf)

![280 pi camera flange pump](./demo_img/280pi/280_pi_camera_flange_pump.png)

[mycobot 280 pi adaptive gripper](./mycobot_description/urdf/mycobot_280_pi/mycobot_280_pi_adaptive_gripper.urdf)
![280 pi adaptive gripper](./demo_img/280pi/280_pi_adaptive_gripper.png)


[mycobot 280 JetsonNano](./mycobot_description/urdf/mycobot_280_jn/mycobot_280_jn.urdf)

![280 jn](./demo_img/280jn/280jn.png)

[mycobot 280 JetsonNano adaptive gripper](./mycobot_description/urdf/mycobot_280_jn/mycobot_280_jn_adaptive_gripper.urdf)
![280 m5 adaptive gripper](./demo_img/280jn/280jn_adaptive_gripper.png)


[mycobot 280 Arduino](./mycobot_description/urdf/mycobot_280_arduino/mycobot_280_arduino.urdf)

![280 jn](./demo_img/280ar/280ar.png)

[mycobot 280 Arduino adaptive gripper](./mycobot_description/urdf/mycobot_280_arduino/mycobot_280_arduino_adaptive_gripper.urdf)
![280 AR adaptive gripper](./demo_img/280ar/280ar_adaptive_gripper.png)


[mycobot 280 x3pi](./mycobot_description/urdf/mycobot_280_x3pi/mycobot_280_x3pi.urdf)

![280 x3pi](./demo_img/280pi/280_pi.png)

[mycobot 280 x5pi](./mycobot_description/urdf/mycobot_280_x5pi/mycobot_280_x5pi.urdf)

![280 x5pi](./demo_img/280pi/280_pi.png)

[mycobot 280 RDKX5](./mycobot_description/urdf/mycobot_280_rdkx5/mycobot_280_rdkx5.urdf)

![280 rdkx5](./demo_img/280rdkx5/280rdkx5.png)

[mycobot 280 RDKX5 adaptive gripper](./mycobot_description/urdf/mycobot_280_rdkx5/mycobot_280_rdkx5_adaptive_gripper.urdf)

![280 rdkx5 gripper](./demo_img/280rdkx5/280rdkx5_adaptive_gripper.png)
