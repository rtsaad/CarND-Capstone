# Udacity Self-Driving Car Engineer Nanodegree

* * *


## Final Project - System Integration

## Team Smart Car Trek

![image alt text](documentationImages/carla.jpg)

### Team Members:
 - Rodrigo Saad (Team Lead) digaots@gmail.com
 - Alexandre Nogueira alexandre.nogueira@gmail.com
 - Pablo Elizalde pelizcar@gmail.com 
 - Simão Luiz Stanislawski Júnior simaoluizjr@gmail.com
 - Alper Sunar alper_sunar@yahoo.com 
 
---
###Waypoint Updater

This node publishes the next LOOKAHEADWPS number of waypoints that are closest to vehicle's current location and are ahead of the vehicle. This node also considers obstacles and traffic lights to set the velocity for each waypoint.

![Waypoint updater (partial)](/documentationImages/waypoint_updater_partial.png)This node subscribes to following topics:

- /base_waypoints: Waypoints for the whole track are published to this topic. This publication is a one-time only operation. The waypoint updater node receives these waypoints, stores them for later use and uses these points to extract the next LOOKAHEADWPS number of points ahead of the vehicle.

- /traffic_waypoint: To receive the index of the waypoint in the base_waypoints list, which is closest to the red traffic light so that the vehicle can be stopped. The waypoint updater node uses this index to calculate the distance from the vehicle to the traffic light if the traffic light is red and the car needs to be stopped.

- /current_pose: To receive current position of vehicle.

- /current_velocity: To receive current velocity of the vehicle which is used to estimate the time the car needs to reach the traffic light’s stop line.


---
### Traffic Light Detection

For the traffic light detection part we have decided to use two classifiers. One classifier to detect the traffic lights in the image, and a second one to detect the status of the traffic light (gree/yellow/red).

For the first classifier we chose to use TensorFlow's Object Detection API. First step was to select one of the pre-trained models that offers the tool. This models have been trained with the [COCO dataset](http://cocodataset.org/#home) and are able to detect many objects out of the box.

We tested several models with different images, taking also into account the performance

![models](/documentationImages/models.png)

We finally decided to go for the MobileNet model.

![timing](/documentationImages/object_detection_timing.png)

Once traffic lights are detected, from the resulting bounding boxes we get the traffic light images that we use in our second classifier.

This classifier follow the same approach that we followed in the Traffic Sign Classifier. We trained a Convolutional Neural Network (CNN) with the udacity simulator and real data images. We used augmentation to increase the number of images for the training. 

1. Input image
![input_image](/documentationImages/red_example.jpg)
2. Traffic light detected

![detection](/documentationImages/red_detected.png)

With more than one detection:

1. Input image
![input_image](/documentationImages/green_example.png)

2. Detections

![traffic_light1](/documentationImages/green_detected_1.png)
![traffic_light2](/documentationImages/green_detected_2.png)

NOTE:
We faced a big issue in case several traffic lights are detected with different status. That issue was adding a lot of clomplexity to the task, since we should have calculated which traffic light is the one that our car should obey. For simplicty we decided to follow the following rules:
 * In case that we have at least a red traffic light detected we return TrafficLight.RED.
 * In case none of the traffic lights is detected as red, we return TrafficLight.UNKNOWN.

 So basically our is reduced to two states: stop and go.

---
### Twist Controller

The twist controller does receive all the arguments that are related to the car properties and the PID controller. It is the software node that is responsible to receive the inputs/arguments from the topics gathered from dbw_node.py and translate it into steering commands to the car.

It also calls the PID controller to be reset and to receive the incoming data from the waypoints. Such node contains the PID parameters that are required to tune the vehicle operation, for the steering operation.
In this script we also set the parameters for the PID controller that has been implemented. This PID parameters were set manually and the values obtained were the best we reached.

### Yaw Controller
The yaw controller is an unchanged node from the original git deploy. The only change in this node is the addition of a check on the current velocity, that increases the accuracy for steering for small angles.


---
This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

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
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

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

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
