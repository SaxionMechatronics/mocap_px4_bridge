# Mocap-PX4 Bridge

This ROS 2 Package takes the Position and Orientation measurements from OptiTrack system and send them to PX4 through the [uXRCE-DDS](https://docs.px4.io/main/en/middleware/uxrce_dds.html) bridge between ROS 2 and PX4.

> [!WARNING]  
> Check that the OptiTrack 'Up axis' is the Z-axis, in Motive the default is Y-axis

## Pre conditions

This package assumes that the OptiTrack system is broadcasting the position and orientation of the robot with the following reference frame convention:

* World frame: X axis Forward, Y-axis Left, Z-axis Up (Again: Check that the OptiTrack 'Up axis' is the Z-axis, in Motive the default is Y-axis).
* Body frame: X axis Forward, Y-axis Left, Z-axis Up.

## Converting to PX4 reference frame convention

Assuming that:

* `P_opti` is the position measurements from OptiTrack 
* `P_px4` is the position sent to PX4
* `q_opti` is the quaternion orientation measurements from OptiTrack
* `q_px4` is the quaternion sent to PX4. Here  

Here is how the frame conversion is carried out:
```python
P_px4.x =   P_optitrack.x​
P_px4.y = - P_optitrack.y​
P_px4.z = - P_optitrack.z​

q_px4.w =   q_px4.w​
q_px4.x =   q_px4.x ​
q_px4.y = - q_px4.y​
q_px4.z = - q_px4.z​
```
