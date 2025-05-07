# Aruco-PP-OPMP
This file is source code about "Marker Position Estimation and Robot Manipulator Control using ROS-based Vision System" article

# Installation

1. install Open-Manipulator-X PKG

2. Install OpenMV and make Camera connection node pkg.

3. Attach Nicla Vision Camera to Open-Manipulator-X

4. Upload Camera Frame program to Nicla Vision

5. Run Open-Manipulator-X, OpenMV and check TF, Node.

6. If verified, run aruco_real.py, pick_real.py files in the aruco_marker_pick_and_place pkg.

# Caution

- OpenMV Version is changed. So you must make your own OpenMV Node pkg.

- This package version is coded with moveit usage. So if you want to use it as is, use moveit together, and for higher IK operations and repetitions, use the IK functions provided by Open-manipulator-X.
