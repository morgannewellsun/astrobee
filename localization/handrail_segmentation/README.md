
### WARNING
### THIS README IS OUT OF DATE
### WARNING

This module contains the code for handrail pose estimation.

### Running handrail ICP localization nodes

(Note: This methodology is messy and should be updated.)

1. Update the reference pointcloud filepath in the `pointcloud_callback` function in `.../handrail_segmentation/src/icp_pose_estimator.py`.
2. Update the model checkpoint path in the handrail_detector.py file
2. Install prerequisites (will not work in a venv; prerequisite structure is messy; `.../handrail_segmentation/requirements.txt` seems to work but still has some weird issues)
2. `roslaunch .../handrail_segmentation/launch/sim_handrail.launch` (this launches the three nodes required for handrail segmentation, masking, and ICP)
3. From another terminal, `./handrail_segmentation/launch_simulation.sh` (will not work in a venv; this launches the Classic Gazebo simulation with Astrobee initialized in a pose such that it is looking at a handrail) 
