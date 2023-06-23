### To run data generation:

1. Update the filepaths and column labels in `.../handrail_segmentation/worlds/plugins/DataGeneration.hh` (specify input column labels in the format {unique_name, pos_x, pos_y, pos_z, rot_euler_x, rot_euler_y, rot_euler_z})
2. Build the `.../handrail_segmentation/worlds/plugins/DataGeneration.cc` plugin (this doesn't get built with the rest of the astrobee project)
3. Modify the hard-coded paths in `.../handrail_segmentation/worlds/iss.world` (one path points to the .so file for DataGeneration.cc, the other points to the desired data output path)
4. `export IGN_GAZEBO_RESOURCE_PATH=.../handrail_segmentation/worlds/models`
5. From `.../handrail_segmentation/worlds/`, run `MESA_GL_VERSION_OVERRIDE=3.3 ign gazebo iss.world` (MESA_GL_VERSION_OVERRIDE=3.3 forces ignition gazebo to use a sufficiently up-to-date version of OpenGL)
6. Click the refresh button in Image Display, then select one of the two "/segmentation_camera/..." topics (this causes the ignition gazebo GUI to subscribe to the sensor, which forces it to run continuously; otherwise, it won't generate any data)
7. Click the run button.

