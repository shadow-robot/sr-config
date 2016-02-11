# Extrinsic Calibration of Multiple Cameras
This package contains the bare-bones of what is necessary to use the industrial_extrinsic_calibration stack to "automatically" calibrate a scene involving multiple cameras.
## Initial Configuration
Principally, there are three things that need to be configured.

### Target
#### Frame
The location of the target is set via the link "target_frame" and the joint which connects it to the world_frame in camera_scene.xacro. This file is found in the urdf directory of this package.
#### Setup
camera_scene_targets.yaml in the yaml directory of this package defines the target(s) we will use, and their location within the target_frame. The best place to look for these are in (industrial_extrinsic_cal)/targets, where you can find both the images from which to make the physical targets, and their corresponding yaml descriptions.

### Camera setup
camera_scene_cameras.yaml in the yaml directory of this package is where we define the cameras whose positions we will be calibrating. The main thing is to make sure the images topics correspond to the setup of the camera driver, and the frames correspond to those defined in camera_scene.xacro. At the moment, they're all called "asus" as that's how they were named in the example.

### Calibration Job
camera_scene_caljob.yaml is where we assocaiate each camera with the target with which it will be calibrated. If a camera can see multiple targets, you can limit the range of interest (in pixels) so that only the desired target is selected.

## Calibrating
First launch you cameras. The file "cameras.launch" is included to launch two of the standard Logitech web-cams we use at shadow. Remeber to set the camera serial numbers to reflect the particular cameras you have.

Then launch the calibration job - camera_scene_cal.launch will do the trick.

Finally, call the service with ```rosservice call /calibration_service "{allowable_cost_per_observation: 0.25}"``` The allowable_cost_per_observation sets how accurate the calibration must be to be accepted. 0.25 is the default, and it seems to work well enough. Maybe we should experiment with this a bit.

The output of the calibration is a new yaml file called "mutable_joint_states.yamlnew". Rename this to just ".yaml" to use the new calibration.

## Using the Calibration
Launching publish_tf.launch will look at the xacro and relevenat yaml files and publish the tfs needed to use the new calibration. Running ```gen_urdf.sh``` in the urdf directory will generate a fixed urdf of the camera setup for using later.
