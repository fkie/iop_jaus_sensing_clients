See [iop_core](https://github.com/fkie/iop_core/blob/master/README.md) for use instructions.

# Interfaces

The repository contains clients designed to control services on IOP complient robot. All client services are based on ```SlaveHandlerInterface``` and use funtionality of [Slave](https://github.com/fkie/iop_core/blob/master/doc/iop_core_packages.md#iop_ocu_slavelib_fkie).  
List of client service plugins in this repository:

[iop_client_digital_video_fkie: DigitalVideoClient](#iop_client_digital_video_fkie-digitalvideoclient)  
[iop_client_range_sensor_fkie: RangeSensorClient](#iop_client_range_sensor_fkie-rangesensorclient)  
[iop_client_still_image_fkie: StillImageClient](#iop_client_still_image_fkie-stillimageclient)  
[iop_client_visual_sensor_fkie: VisualSensorClient](#iop_client_visual_sensor_fkie-visualsensorclient)

## _iop_client_digital_video_fkie:_ DigitalVideoClient

Forwards ```Play``` and ```Stop``` of a resource to [DigitalVideo](https://github.com/fkie/iop_jaus_sensing#iop_digital_video_fkie-digitalvideo). To get the digital enpoints use [DigitalResourceClient](https://github.com/fkie/iop_platform#iop_client_digital_resource_fkie-digitalresourceclient)

#### Parameter:

> None

#### Publisher:

> None

#### Subscriber:

_dv_resource_id (std_msgs::UInt16)_

> Resource ID of current played stream. ```65535``` for ```Stop```.

## _iop_client_range_sensor_fkie:_ RangeSensorClient

Requests data from [RangeSensor](https://github.com/fkie/iop_jaus_sensing#iop_range_sensor_fkie-rangesensor) and publishes also tf for each sensor if ```ReportSensorGeometricProperties``` available.

#### Parameter:

_hz (int_ Default: 5.0)

> Sets how often the reports are requested. If [use_queries](https://github.com/fkie/iop_core/blob/master/doc/iop_core_packages.md#parameter) is ```True``` hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```use_queries``` is ```False``` an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created.

_tf_frame_robot (str_ Default: "base_link")

> Robot tf frame used to publish a tf for each sensor.

#### Publisher:

_tf (geometry_msgs::TransformStamped)_

> Tf for position of the range sensor on the robot.

_{sensor name} (sensor_msgs::LaserScan)_

> Creates for each sensor a publisher.

#### Subscriber:

> None

## _iop_client_still_image_fkie:_ StillImageClient

Requests data from [StillImage](https://github.com/fkie/iop_jaus_sensing#iop_still_image_fkie-stillimage). Tries also to get names for sensor ID's from [VisualSensor](https://github.com/fkie/iop_jaus_sensing#iop_visual_sensor_fkie-visualsensor). This names are used as topic names.

#### Parameter:

_hz (int_ Default: 1.0)

> Sets how often the reports are requested. If [use_queries](https://github.com/fkie/iop_core/blob/master/doc/iop_core_packages.md#parameter) is ```True``` hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```use_queries``` is ```False``` an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created.

_use_id_for_topics (bool_ Default: true)

> Allows to use ID if no name for sensor is available. In other case a warning will be printed!

#### Publisher:

_{sensor name} (image_transport::ImageTransport)_

> Creates for each sensor a publisher.

#### Subscriber:

> None

## _iop_client_visual_sensor_fkie:_ VisualSensorClient

Gets names for sensor ID's from [VisualSensor](https://github.com/fkie/iop_jaus_sensing#iop_visual_sensor_fkie-visualsensor).

#### Parameter:

> None

#### Publisher:

_visual_sensor_names (iop_msgs_fkie::VisualSensorNames)_

> Publishes the discovered pairs of ID and name.

#### Subscriber:

> None
