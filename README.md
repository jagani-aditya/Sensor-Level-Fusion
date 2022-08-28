# Sensor-Level-Fusion-with-Extended-Kalman-Filter
Most autonomous driving cars are equipped with Lidar and Radar. However the outputs of those two are different, the output of Lidar is positions of objects in cartesian coordinates whereas Radar gives out the position and velocity of the objects in polar coordinates. The Extended Kalman Filter is utilized as it can fuse non-linear data, in this case the data from cartesian coordinates and polar coordinates. To estimate the non-linear measurement, the Jacobian matrix is introduced. 

As the results below, the EKF functions fuses linear Lidar with non-linear Radar data.

<p align="center">
  <img src="/images/ekf1.png"  />
</p>

