# Teleop control
Install this [package](http://wiki.ros.org/teleop_twist_keyboard).
use u and o to change direction of the steering angle and follow the guide on terminal when you run the code to change velocity and steering control.
Run estimation.py and record the cmd_delta topic.
twist messgae of the cmd_topic stores all the measurements at that point of time needed to estimate the stiffness parameters.

# Parameter estimation
Play the bag file and run either estimate1 or estimate2. Change the data_len global variable to vary the time for which it collects the data on which linear fitting will be done. 

