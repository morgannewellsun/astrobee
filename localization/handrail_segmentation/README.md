This module contains the code for handrail pose estimation. The worlds directory contains the ignition gazebo ported world used to generate the training data for the segmentation model as well as the plugins written. Furthermore, pytorch_mrcnn contains the training and testing scripts utilized for the mask rcnn. Inside launch you can see the launch file for simulation and for bag files recorded in granite lab. Note that handrail_detector.py inside of src is where I put effort for optimizing and got the detections up to 0.3 Hz.

### Running handrail ICP localization nodes

(Note: This methodology is messy and should be updated.)

1. Update the reference pointcloud filepath in the `pointcloud_callback` function in `.../handrail_segmentation/src/icp_pose_estimator.py`.
2. Install prerequisites (will not work in a venv; prerequisite structure is messy; `.../handrail_segmentation/requirements.txt` seems to work but still has some weird issues)
2. `roslaunch .../handrail_segmentation/launch/sim_handrail.launch` (this launches the three nodes required for handrail segmentation, masking, and ICP)
3. From another terminal, `./handrail_segmentation/launch_simulation.sh` (will not work in a venv; this launches the Classic Gazebo simulation with Astrobee initialized in a pose such that it is looking at a handrail) 
