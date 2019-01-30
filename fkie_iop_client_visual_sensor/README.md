This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _fkie_iop_client_visual_sensor:_ VisualSensorClient

Gets names for sensor ID's from [VisualSensor](https://github.com/fkie/iop_jaus_sensing/blob/master/fkie_iop_visual_sensor/README.md).

#### Parameter:

_hz (float_ , Default: 0.0)

> Sets how often configuration reports are requested. If [use_queries](https://github.com/fkie/iop_core/blob/master/fkie_iop_ocu_slavelib/README.md#parameter) is ```True``` hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```use_queries``` is ```False``` an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created.


#### Publisher:

_visual_sensor_names (fkie_iop_msgs::VisualSensorNames)_

> Publishes the discovered pairs of ID and name.

_`ID`/pwr_state (std_msgs::Bool)_

> Publishes the current power state of the sensor. Only available if remote sensor supports states.

_`ID`/zoom_level (std_msgs::Bool)_

> Publishes the current zoom level for the sensor. Only available if remote sensor supports zoom modes.

_`ID`/fov_horizontal (std_msgs::Float32)_

> Publishes the current horizontal field of view for the sensor.

_`ID`/fov_vertical (std_msgs::Float32)_

> Publishes the current vertical field of view for the sensor.


#### Subscriber:

_`ID`/cmd_pwr_state (std_msgs::Bool)_

> Sets the new power state for the sensor. Only available if remote sensor supports states.

_`ID`/cmd_zoom_level (std_msgs::Bool)_

> Sets the new zoom level for the sensor. Only available if remote sensor supports zoom modes.

