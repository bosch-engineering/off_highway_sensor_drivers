# off_highway_radar

The off_highway_radar package provides a receiver node to receive and decode CAN frames of the Bosch
Radar Off-Highway into ROS messages - it implements an `off_highway_can::Receiver`. Furthermore, the
package provides a sender node to encode and send needed radar input data as CAN frames - an
implementation of an `off_highway_can::Sender`.

Further information on the [Bosch Radar Off-Highway](https://www.bosch-mobility-solutions.com/en/solutions/assistance-systems/radar-systems-ohw), it's inputs, outputs and how they can be
interpreted can be found in the corresponding datasheet or the Technical Customer Documentation (TCD) of the sensor
system.

## Supported devices

| **Device name** | **Part Number** | **Description** | **Datasheet** |
| -| - | - | - |
| Radar Off-Highway | - F037.000.127 (series - from software version F037SW0157V06.0001 ) <br> - F037.B00.575-04 (sample - from software version F037SW0116V05.0004) | - Radar sensor with up to 40 objects (filtered & unfiltered)<br> - Object output on automotive CAN is supported | [Link](https://www.bosch-mobility.com/media/global/solutions/off-highway-and-large-engines/driver-assistance-systems/radar-systems/beg_summary_radaroffhighway_en_rgb_150dpi_20210121.pdf)|

Contact: [**off-highway.beg@bosch.com**](mailto:off-highway.beg@bosch.com?subject=off_highway_sensor_drivers%20Radar%20OHW%20Sensors)

## Nodes

### Receiver

The radar receiver decodes CAN frames into an object list, manages the current list and publishes it
cyclically.

All received messages are checked for their cyclic redundancy check (CRC), rolling message counter
and age (message not older than parameter `allowed_age`). If any of these checks do not succeed the
received message is not further processed and skipped.

The relevant radar CAN frame IDs to process are specified by the `object_base_id` and `info_id`
parameters. They should correspond to the first object frame ID and the info frame ID of the radar
CAN node and need to be adapted for the specific bus setup. If multiple radar sensors need to be
decoded on the same bus just launch multiple receiver nodes with individual `*_id` parameters.

The object list is published as a list of radar objects or as a point cloud and contains up to 40
objects (sensor limit). Only valid objects in the list are published (valid flag set).

The radar zone data is not processed in this version.

If the receiver node does not receive anything within a configured period (parameter `timeout`), it
will publish a diagnostic error on `/diagnostics`. Also, the sensor information CAN frame gets
published and is checked for the diagnostic status of the radar. On error, the node will publish a
diagnostic error on `/diagnostics`.

#### Subscribed Topics

* **from_can_bus
  ([`can_msgs/Frame`](http://docs.ros.org/en/noetic/api/can_msgs/html/msg/Frame.html))**
  * Radar CAN frames to decode

#### Published Topics

* **objects
  ([`off_highway_radar_msgs/Objects`](../off_highway_radar_msgs/msg/Objects.msg))**
  * Update Rate: configurable with `publish_frequency`
  * Contains current object list of radar as custom message containing the same information as each
    object's CAN frame. Mapping to the respective CAN frame is done via the `id` field.
* **objects_pcl
  ([`sensor_msgs/PointCloud2`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html))**
  * Update Rate: configurable with `publish_frequency`
  * Contains current object list of radar as point cloud.
* **info
  ([`off_highway_radar_msgs/Information`](../off_highway_radar_msgs/msg/Information.msg))**
  * Update Rate: On each sensor information CAN frame
  * Contains current sensor information message containing the same information as the respective
    CAN frame.
* **/diagnostics
  ([`diagnostic_msgs/DiagnosticArray`](http://docs.ros.org/en/noetic/api/diagnostic_msgs/html/msg/DiagnosticArray.html))**
  * Update Rate: On each sensor information CAN frame or if receiver timed out periodically with
    timeout period
  * Diagnostic status contains statuses from the latest received sensor information and timeout
    status.

#### Parameters

See [receiver_params.yaml](config/receiver_params.yaml).

### Sender

The radar sender listens for ROS twist messages as input, encodes them as CAN frames and publishes
those as ROS messages.

Both radar input CAN signals only have a limited range:

* Ego velocity signal: [-81.92 m/s, 81.9175 m/s]
* Yaw rate signal: [-163.84 deg/s, 163.83 deg/s]

Input ROS messages with values outside of these ranges are discarded by the sender and not encoded /
published. Furthermore, if the ROS twist message is too old (parameter `allowed_age`), it is
discarded, too.

The relevant radar CAN frame IDs to publish the forward velocity and yaw rate are specified by the
`ego_velocity_id` and `yaw_rate_id` parameters. These parameters need to correspond to the ego
velocity and yaw rate CAN frame ID for the specific bus setup. If multiple radar systems are
connected to the same bus, a single radar sender is sufficient as long as these radar systems expect
the ego velocity and yaw rate CAN messages on the same IDs (in contrast to multiple receivers).

If the sender node does not receive anything valid within a configured period (parameter `timeout`),
it will publish a diagnostic error on `/diagnostics`.

#### Subscribed Topics

* **velocity
  ([`geometry_msgs/TwistStamped`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistStamped.html))**
  * Forward velocity and yaw rate is sent to the radar for compensating velocity readings

#### Published Topics

* **to_can_bus
  ([`can_msgs/Frame`](http://docs.ros.org/en/noetic/api/can_msgs/html/msg/Frame.html))**
  * Encoded radar CAN frames
* **/diagnostics
  ([`diagnostic_msgs/DiagnosticArray`](http://docs.ros.org/en/noetic/api/diagnostic_msgs/html/msg/DiagnosticArray.html))**
  * Update Rate: 1 Hz or if sender timed out periodically with timeout period
  * Diagnostic status only contains timeout status

#### Parameters

See [sender_params.yaml](config/sender_params.yaml).

## Launch files

* **[receiver_launch](launch/receiver_launch.py)**: Starts the receiver with the given parameters.
  * Arguments:
    * **params**: Path to ROS YAML parameter file to load for receiver. If not provided, default
      parameters from this package are loaded.
* **[sender_launch](launch/sender_launch.py)**: Starts the sender with the given parameters.
  * Arguments:
    * **params**: Path to ROS YAML parameter file to load for sender. If not provided, default
      parameters from this package are loaded.
