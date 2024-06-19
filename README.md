# README

This repository is based on the duckietown template, given in (https://github.com/duckietown/template-basic).

The key elements of this repository, which will be discussed are the ROS nodes added with a basic explanation about its working and the launcher used

# packages

The packages can be found in the packages directory. 

The first package to be discussed is the nn_model package/directory.
The directory consists of a python script called model.py, where the model is initialised and the prediction is performed on images.
The model which is used can also be configured in here. The model locations itself are in the weights subdirectory

The second package to be discussed is the object_detection package/directory
Here images are being received from the duckiebot, where first image distortion and rectification is performed, whereas
afterward object detection is performed based on the nn_model package and some filtering is applied. Then depth estimation is performed, whereafter
measurements are send to the path planning node.

The final package is path planning
Here the controls to the actuator are determined based on the incoming data from the object detection node.
It also consists of a small statemachine, in order to succesfully drive to an object, pick it up and drop it off
at a certain other object. This sequence can easily be changed dependent on a list defined.

# launcher

A single launcher is used to run everything located in the launchers directory, for which the default.sh launcher is used.
to run the code, one could use the command: 'dts devel run -R db2 -X'

# dependencies

the main dependencies are listed in dependencies-py3.txt
