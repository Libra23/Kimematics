# Kimematics
Forward & Inverse kinematics library for ESP32 and Standard C++
## Description
Compute end-effector position & rotation from joint angle, 
and determine joint angle using Singularity low-sensitive motion resolving
## Requirement
- ESP32(Arduino)
  - https://github.com/tomstewart89/BasicLinearAlgebra
- Linux or other C++ environment
  - http://eigen.tuxfamily.org/index.php?title=Main_Page
## Demo
### Forword Kinematic
```
Joint q;
q << 45.0, 0.0, 90.0;
q *= DEG_TO_RAD;
Affine3d tip_trans;
kinematic_.Forward(q, Affine3d::Identity(), tip_trans);
// print tip_trans
```
### Inverse Kinematic
```
Joint q_pre = q;
tip_trans.translation() += Vector3d(20.0 * sin(2 * M_PI * t), 20.0, 20.0); // Update only position
kinematic_.Inverse(tip_trans, Affine3d::Identity(), q_pre, q);
// print q
```