# off_highway_premium_radar_msgs

This package provides the ROS messages for the off_highway_premium_radar package and maps the UDP
interface of the premium radar to ROS messages.

The naming of messages and fields is aligned to the UDP interface description but updated (e.g.,
written verbosely) to adhere ROS naming conventions.

## Messages

* [LocationDataHeader](msg/LocationDataHeader.msg): Mapped Location Data/LocData_Header_i from UDP
  interface.
* [SensorFeedback](msg/SensorFeedback.msg): Mapped Sensor Feedback from UDP interface.
* [SensorStateInformation](msg/SensorStateInformation.msg): Mapped Sensor State Information from UDP
  interface.
* [SensorBroadcast](msg/SensorBroadcast.msg): Mapped Sensor Broadcast from UDP interface.
* [LocationAttributes](msg/LocationAttributes.msg): Mapped Location Attributes from UDP interface.
* [SensorDtcInformation](msg/SensorDtcInformation.msg): Mapped Sensor DTC Information from UDP
  interface.
* [EgoVehicleInput](msg/EgoVehicleInput.msg): Mapped Ego Vehicle Input from UDP interface.

Rest of the defined messages are sub messages in the above top-level nested messages to replicate
the nested description of the UDP packages in the UDP interface.

> **Note:** LocationData is provided as
> [`sensor_msgs/msg/PointCloud2`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html).

## Services

* [MeasurementProgram](srv/MeasurementProgram.srv): Mapped Measurement Program Input from UDP
  interface.
* [SensorModeRequest](srv/SensorModeRequest.srv): Mapped Sensor Mode Request Input from UDP
  interface.
