# CarND3 Programming a Real Self-Driving Car

## Description

**This is our capstone project result of Udacity self-driving car nanodegree (CarND) term 3. It's required to team up to implement a self-driving car system including perception, planning and control modules. It's first tested using Udacity simulater and then actually runs on Udacity self-driving car, Carla ! For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).**

![alt text][image1]

**The following demonstrates test results using Udacity simulator:** 
![alt text][image3] ![alt text][image4]


* Udacity self-driving car nanodegree (CarND) :

  https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013

* Udacity simulator of this project  :
  
  https://github.com/udacity/CarND-Capstone/releases

[//]: # (Image References)
[image1]: ./imgs/1.jpg
[image2]: ./imgs/2.png
[image3]: ./imgs/3.gif
[image4]: ./imgs/4.gif

## Team Members

|     Name     |     E-mail    | Contribution  |
| ------------ | ------------- | ------------- |
| Raymond Chen | raymonduchen@gmail.com | Team lead, waypoint updater |
| Daehun Kim | the614@gmail.com | Traffic light detection |
| Gregory Van | gvan316@gmail.com | Traffic light detection |
| Hsin-Wen Chang | bow1226@gmail.com | DBW (drive-by-wire) node |
| Yi-Ching Chung | tmcake@gmail.com | DBW (drive-by-wire) node |


## System Architecture

![alt text][image2]

The whole systems can be divided into perception, planning, control system and car/simulater. Besides modules provided by Udacity, we implemented waypoint updater, traffic light detection and DBW node as indicated in the above image.

### Waypoint Updater
It sets a series of look-ahead waypoints starting from current vehicle position for drive-by-wire module to follow. This module generates different waypoints based on traffic light detection results (green, yellow or red light). 

In red light case, the car will decelerate gradually to 1 kph starting from current velocity within 10 ~ 40 mile before traffic light. Otherwise, it will keep stop. In yellow or green light case, the car will accelerate and then keep the target velocity (default 20kph).

Overall, the car will follow waypoints and try to keep constant target velocity. When seeing red light in front, it will decelerate gradually and finally stop in front of traffic light.

### Traffic Light Detection
SSD Mobile net is used for detecting traffic light. Both images from simulater and real life images are trained and fine tuned, respectively.

### DBW (Drive-by-wire) Node
For throttle control, it uses PID controller. Parameters are set by trial and error based on different velocity. 




## Setup

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/SmartDriveTeam/SmartDrive.git
```

2. Install python dependencies
```bash
cd SmartDrive
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
chmod +x -R src/
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd SmartDrive/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
