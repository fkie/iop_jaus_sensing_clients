This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _iop_client_range_sensor_fkie:_ RangeSensorClient

Requests data from [RangeSensor](https://github.com/fkie/iop_jaus_sensing/blob/master/iop_range_sensor_fkie/README.md) and publishes also tf for each sensor if ```ReportSensorGeometricProperties``` available.

#### Parameter:

_hz (int_ Default: 5.0)

> Sets how often the reports are requested. If [use_queries](https://github.com/fkie/iop_core/blob/master/iop_ocu_slavelib_fkie/README.md#parameter) is ```True``` hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```use_queries``` is ```False``` an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created.

_tf_frame_robot (str_ Default: "base_link")

> Robot tf frame used to publish a tf for each sensor.

#### Publisher:

_tf (geometry_msgs::TransformStamped)_

> Tf for position of the range sensor on the robot.

_{sensor name} (sensor_msgs::LaserScan)_

> Creates for each sensor a publisher.

#### Subscriber:

> None
