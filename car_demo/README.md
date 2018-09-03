# Demo of Prius in ROS/GAZEBO

This is a simulation of a Prius in [gazebo 8](http://gazebosim.org) with sensor data being published using [ROS kinetic](http://wiki.ros.org/kinetic/Installation)
The car's throttle, brake, steering, and gear shifting are controlled by publishing a ROS message.
A ROS node allows driving with a gamepad or joystick.

# Requirements

* An X server (should be available by default)
* [Docker](https://www.docker.com/get-docker)
* [nvidia-docker](https://github.com/NVIDIA/nvidia-docker/wiki/Installation)

# Building (If using DockerFile)

Run the script `build_demo.bash`.
It builds a docker image with the local source code inside.

```
$ cd car_demo
$ ./build_demo.bash
```

# Pulling Image (If you are lazy to build one)

Pull the image either from author's dockerhub repo or original repo. 
Original repo will contain only the vanilla layers. Author might add some layers to the image as required

```
$ sudo docker pull system32/car_demo
```

OR

```
$ sudo docker pull osrf/car_demo
```

# Running

Use the script `run_demo.bash` to run the demo.

```
$ ./run_demo.bash
```

If you want to run RVIZ on your own PC, then accordingly modify the the container or image, whichever seems easy.


An [RVIZ](http://wiki.ros.org/rviz) window will open showing the car and sensor output.
A gazebo window will appear showing the simulation.
Either use the controller to drive the prius around the world, or click on the gazebo window and use the `WASD` keys to drive the car.


After a successfull launch, you should be able to see rostopics upon listing on the host terminal. Also, check echo for /clock. If no messages are received, then there's a problem with your launch commands. If the simulator runs without any issue, you can go ahead with subscribing/publishing to topics. 

