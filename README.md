# Objective
visualizing of 6dof imu (mpu6050) data \
additionally derived and implemented kalman filter 

## visualize_raw_data.ipynb
read out and visualize data from mpu_6050_i2cdevlib_raw.ino, quaternion math and transformations included.

## mpu_6050_...
these folders include different implementations to read the data from the mpu6050 by an arduino starting with a very basic example up to a low-level ic2 and uart implementation.

## kalman_filter_basics.ipynb
derivation of kalman filter and visualization in 2D (position px/py and velocity vx/vy with control input ax/ay to stay in circular path)
