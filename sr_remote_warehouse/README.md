# Shadow Pose Warehouse

This package allows connection to a mongodb warehouse for storing robot poses for use with the Shadow Robot Hand. 

It provides a cloud alternative to local a mongo warehouse as provided by [mongodb](http://wiki.ros.org/mongodb). As for the local warehouse, the remote one also provides storage for constraints and planning scenes. These features aren't used by our code, and so will be left unexplained.


## Launching

The easiest way to launch the database connection is with ```roslaunch sr_remote_warehouse remote_warehous.launch```. This will add relevant parameters to the param server, then launch a ```warehouse_ros warehouse_services``` node to provide service access to robot states saved in the database. See below for further explanation of this. Add ```launch_servies:=false``` to the launch command if you don't want the services for some reason.

The connection can also be launched for existing shadow robot moviet configurations:
* ```right_sr_ur10_moveit_config```
* ```left_sr_ur10_moveit_config```
* ```sr_hand_moveit_config```
by using ```roslaunch moveit_config_package_name default_warehouse_db.launch remote_warehouse:=true```

## Connection details
Default access is read only, using the authentication details stored in ```sr_remote_warehouse/launch/remote_warehouse.yaml```. If you want write access for some reason, [contact us](mailto:software@shadowrobot.com).
## Accessing Poses

The poses stored in the database can be most easily accessed using the following services, launched by default with the database connection.

*```/list_robot_states``` returns a list of the names of the robot states available in the db. If you provide a ```robot``` argument, only posess for the named robot are returned.
*```/has_robot_state``` returns a bool describing if the named state is available. If ```robot``` is provided, the service returns true only if the named pose is available, and for the named robot.
*```/get_robot_state``` returns a ```moveit_msgs/RobotState``` containing the saved pose. The ```robot``` argument can be used to filter poses only for the named robot as before.

Support for transpartent access from the sr_robot_commander, the sr_grasp_controller and eventually directly frmo the move_group_commander. Watch this space for further details.

If the database connection has been launched, the rviz motion planning plugin can connect in the same way as it would to a local db, although obviously read only access.

## Cloning Locally

The facility to clone the warehouse to be used locally is also scheduled to be added soon.
