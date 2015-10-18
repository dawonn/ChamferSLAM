
This repository contains a reference implementation of ChamferSLAM. 

This algorithm has been submitted to IEEE Transactions on Robotics for 
publication.

ChamferSLAM is a SLAM front-end which processes 2D lidar sensor data to 
build a map of an unknown enviornment, and then provides the location of the
sensor in that map. ChamferSLAM is unique in that it is conceptually simple
to implement and computationally efficient, processing each scan in real 
time from a Hokuyo Lidar scanner.

The dataset format is a CSV file with the raw data from the sensors. You 
will likely need to write your own data import utility based on your sensor.


Files:

setup.m 
    Is used to load a dataset and run ChamferSLAM once.

SLAM.m 
    The main loop of the SLAM algorithm, this script processes each lidar
    scan and produces a map and sensor path.

chamferMatch.m
    Matches a lidar scan with a map.

clusterRun.m
    Runs each dataset on the HPC facilities at Michigan Tech.

clusterRun.sh
    Queue Submission script for the HPC facilities at Michigan Tech.

util/*.m 
    Various utility scripts such as a log file reader for each sensor.



