# Sensor-Level-Fusion

## Project Description
Most autonomous driving cars are equipped with Lidar and Radar. However the outputs of those two are different, the output of Lidar is positions of objects in cartesian coordinates whereas Radar gives out the position and velocity of the objects in polar coordinates. The Extended Kalman Filter is utilized as it can fuse non-linear data, in this case the data from cartesian coordinates and polar coordinates. To estimate the non-linear measurement, the Jacobian matrix is introduced. 

As the results below, the EKF functions fuses linear Lidar with non-linear Radar data.

## Demonstration

<p align="center">
  <img src="ekf_sensorfusion.png" width="66%" />
</p>

## Platform
* MATLAB

## Implementation
 
Navigate to the ```Sensor-Level-Fusion``` folder

Open ```main.m``` file and ```RUN``` it in MATLAB workspace.
