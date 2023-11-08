# How to transform point cloud

## 1 Introduction

Note:
+ Before reading this document, please be sure that you have read the [Online connect LiDAR](how_to_decode_with_online_lidar.md) or [Offline decode pcap bag](how_to_decode_with_pcap_file.md).
+ This function is only for test purpose, and it costs much CPU resources, so **never, never enable this function in your released products**.

This document illustrate how to transform the point cloud to a different position with the built-in trasform function.

The rotation order of the transformation is **yaw - pitch - row**. The unit of x, y, z, is ```m```, and the unit of roll, pitch, yaw, is ```radian```.

## 2 Steps

### 2.1 Compile

To enable the transformation function, compile the driver with the option ENABLE_TRANSFORM=ON.

```bash
cmake -DENABLE_TRANSFORM=ON ..
```

### 2.2 Config parameters

Configure the transformation parameters. These parameters' default value is ```0```.  

Below is an example with x=1, y=0, z=2.5, roll=0.1, pitch=0.2, yaw=1.57. 

```c++
RSDriverParam param;                            ///< Create a parameter object
param.input_type = InputType::ONLINE_LIDAR;     /// get packet from online lidar
param.input_param.msop_port = 6699;             ///< Set the lidar msop port number, the default is 6699
param.input_param.difop_port = 7788;            ///< Set the lidar difop port number, the default is 7788
param.lidar_type = LidarType::RS16;             ///< Set the lidar type. Make sure this type is correct

param.decoder_param.transform_param.x = 1;	  ///< unit: m
param.decoder_param.transform_param.y = 0;	  ///< unit: m
param.decoder_param.transform_param.z = 2.5;	  ///< unit: m
param.decoder_param.transform_param.roll = 0.1; ///< unit: radian
param.decoder_param.transform_param.pitch = 0.2;///< unit: radian
param.decoder_param.transform_param.yaw = 1.57; ///< unit: radian

```

