This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _fkie_iop_client_still_image:_ StillImageClient

Requests data from [StillImage](https://github.com/fkie/iop_jaus_sensing/blob/master/fkie_iop_still_image/README.md). Tries also to get names for sensor ID's from [VisualSensor](https://github.com/fkie/iop_jaus_sensing/blob/master/fkie_iop_visual_sensor/README.md). This names are used as topic names.

#### Parameter:

_hz (int_ Default: 1.0)

> Sets how often the reports are requested. If [use_queries](https://github.com/fkie/iop_core/blob/master/fkie_iop_ocu_slavelib/README.md#parameter) is ```True``` hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```use_queries``` is ```False``` an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created.

_use_id_for_topics (bool_ Default: true)

> Allows to use ID if no name for sensor is available. In other case a warning will be printed!

#### Publisher:

_{sensor name} (image_transport::ImageTransport)_

> Creates for each sensor a publisher.

#### Subscriber:

> None