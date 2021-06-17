For users using **Windows**, this document contains the instructions for quick start:
1. Install [**virtual box**](https://www.virtualbox.org/wiki/Downloads) (Essential because we need port forwarding, which is not provided by VMWARE free edition)
2. Import the [virtual image](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/Udacity_VM_Base_V1.0.0.zip) to virtual box, the detail instruction can be found from the [class](https://classroom.udacity.com/nanodegrees/nd013/parts/b9040951-b43f-4dd3-8b16-76e7b52f4d9d/modules/85ece059-1351-4599-bb2c-0095d6534c8c/lessons/23b85cf1-bc20-469e-b34b-d321e677d91c/concepts/8c742938-8436-4d3d-9939-31e40284e7a6). The Ros Kinetic version already installed.
3. Setting **port forwarding** to `4567` by following this [instruction](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Port+Forwarding.pdf) (Essential or the simulator can not communicate with virtual environment)
4. clone this repo and open `requirements.txt`. Delete `keras==2.0.8` and `tensorflow==1.3.0` at line # 10 and # 11 because this is for the optional challenage and which will not be use currently(see note below).
5. follow the [README](https://github.com/kaorusha/CarND-Capstone/blob/project/README.md#usage) Usage:
#### Install python dependencies
```sh
cd CarND-Capstone
sudo pip install -r requirements.txt
```
#### Make and run styx
```sh
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
6. Run the simulator in **Windows**, because of the port forwarding, the simulator and the ros topics are communicating
7. Here is the result video (10x faster than original video clip)
### note 
An Optional challenge is using SSD architecture to process the incoming camera image and give new velocity of waypoints command when there is a red traffic light. This is not completed because the environment setting is complicated using a VM. For ones that still want to implement, here are some reference:
* [vatsl's repo](https://github.com/vatsl/TrafficLight_Detection-TensorFlowAPI) including lebeled training data
* [additional training data](https://www.uni-ulm.de/en/in/driveu/projects/driveu-traffic-light-dataset/)
* [yuki678's repo](https://github.com/yuki678/driving-object-detection) is a detail step instruction for installing compatible tensorflow and for this challenge using tensorflow's [object detection API](https://github.com/tensorflow/models/tree/master/research/object_detection)

Please remember to cite their works as reference.

Additionally the class lab need installing Anaconda, the [instructions](https://codertw.com/%E7%A8%8B%E5%BC%8F%E8%AA%9E%E8%A8%80/402614/) to install compatible version for ubuntu 16.04 
