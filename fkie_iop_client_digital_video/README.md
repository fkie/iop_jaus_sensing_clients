This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _fkie_iop_client_digital_video:_ DigitalVideoClient

Forwards ```Play``` and ```Stop``` of a resource to [DigitalVideo](https://github.com/fkie/iop_jaus_sensing/blob/master/fkie_iop_digital_video/README.md). To get the digital enpoints use [DigitalResourceClient](https://github.com/fkie/iop_platform/blob/master/fkie_iop_client_digital_resource/README.md)

#### Parameter:

> None

#### Publisher:

> None

#### Subscriber:

_dv_resource_id (std_msgs::UInt16)_

> Resource ID of current played stream. ```65535``` for ```Stop```.