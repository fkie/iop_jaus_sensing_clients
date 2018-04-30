This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _iop_client_digital_video_fkie:_ DigitalVideoClient

Forwards ```Play``` and ```Stop``` of a resource to [DigitalVideo](https://github.com/fkie/iop_jaus_sensing/blob/master/iop_digital_video_fkie/README.md). To get the digital enpoints use [DigitalResourceClient](https://github.com/fkie/iop_platform/blob/master/iop_client_digital_resource_fkie/README.md)

#### Parameter:

> None

#### Publisher:

> None

#### Subscriber:

_dv_resource_id (std_msgs::UInt16)_

> Resource ID of current played stream. ```65535``` for ```Stop```.