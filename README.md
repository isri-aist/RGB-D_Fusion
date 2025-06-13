# RGB-D Immersion

This project uses Unreal Engine coupled with [rclUE](https://rclue.readthedocs.io/en/latest/) to gather information from ROS2 Topics where the [Azure Kinect ROS Driver](https://github.com/microsoft/Azure_Kinect_ROS_Driver) publishes sensor data from the [Azure Kinect Developer Kit](https://azure.microsoft.com/en-us/services/kinect-dk/).

This repository uses the [Azure Kinect Sensor SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK) to communicate with the Azure Kinect and the [Azure Kinect ROS Driver](https://github.com/microsoft/Azure_Kinect_ROS_Driver) to publish data on the network.

## Features

This project uses a variety of sensor data to perform multiple operations, including:

- Display of the RGB Stream
- Custom Sphere to match Depth camera Intrinsic Parameters
- Rectified color Images in the depth camera resolution
- Depth Simulation using Displacement
- A User Interface to control the view

## Requierements

This project has been tested in a specific environment:

- Ubuntu 22.04
- Unreal Engine 5.1.0
- ROS2 Humble
- SteamVR

## Status

This project is provided as a starting point for using the Azure Kinect ROS Driver with Unreal Engine and Depth Simulation using the Azure Kinect datas.

## Known issues

The plugin rclUE used for this project brings some crashes to the Editor whenever a Subscriber can't access a Topic.
To solve this, the source code of the plugin as been modified to avoid being blocking.
