# off_highway_general_purpose_radar

The off_highway_general_purpose_radar package provides a receiver node to receive and decode CAN
frames of the Bosch General Purpose Radar Off-Highway (GPR) into ROS messages - it implements an
`off_highway_can::Receiver`.

Further information on the [Bosch General Purpose Radar Off-Highway (GPR)](https://www.bosch-mobility-solutions.com/en/solutions/assistance-systems/radar-systems-ohw/), it's inputs, outputs and
how they can be interpreted can be found in the corresponding datasheet or the Technical Customer Documentation (TCD) of the sensor system.

## Supported devices

| **Device name** | **Part Number** | **Description** | **Datasheet** |
| -| - | - | - |
| General Purpose Radar Off-Highway (GPR) | - F037.000.100 (series) <br> - F037.B00.255-11 (sample) | - Radar sensor with up to 48 target reflections<br> - Target output on automotive CAN is supported | [Link](https://www.bosch-mobility.com/media/global/solutions/off-highway-and-large-engines/driver-assistance-systems/radar-systems/beg_summary_gerneral_purpose_radaroffhighway_en_rgb_150dpi_20210121.pdf) |

Contact: [**off-highway.beg@bosch.com**](mailto:off-highway.beg@bosch.com?subject=off_highway_sensor_drivers%20Radar%20Sensors)

## Nodes

### Receiver

The radar receiver decodes CAN frames into a target list, manages the current list and publishes
it cyclically.

All received messages are checked for their cyclic redundancy check (CRC), rolling message counter
and age (message not older than parameter `allowed_age`). If any of these checks do not succeed the
received message is not further processed and skipped.

The relevant radar CAN frame IDs to process are specified by the `target_base_id` and `info_id`
parameters. They should correspond to the first target frame ID and the info frame ID of the radar
CAN node and need to be adapted for the specific bus setup. If multiple radar sensors need to be
decoded on the same bus just launch multiple receiver nodes with individual `*_id` parameters.

The target list is published as a list of radar targets or as a point cloud and contains up to
48 targets (sensor limit). Only valid targets in the list are published (measured flag set).

If the receiver node does not receive anything within a configured period (parameter `timeout`), it
will publish a diagnostic error on `/diagnostics`. Also, the sensor information CAN frame gets
published and is checked for the diagnostic status of the radar. On error, the node will publish a
diagnostic error on `/diagnostics`.

#### Subscribed Topics

* **received_messages
  ([`can_msgs/Frame`](http://docs.ros.org/en/noetic/api/can_msgs/html/msg/Frame.html))**
  * Radar CAN frames to decode

#### Published Topics

* **targets
  ([`off_highway_general_purpose_radar_msgs/Targets`](../off_highway_general_purpose_radar_msgs/msg/Targets.msg))**
  * Update Rate: configurable with `publish_frequency`
  * Contains current target list of radar as custom message containing the same information as
    each target's CAN frame. Mapping to the respective CAN frame is done via the `id` field.
* **targets_pcl
  ([`sensor_msgs/PointCloud2`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html))**
  * Update Rate: configurable with `publish_frequency`
  * Contains current target list of radar as point cloud.
* **info
  ([`off_highway_radar_msgs/Information`](../off_highway_general_purpose_radar_msgs/msg/Information.msg))**
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

## Launch files

* **[receiver_launch](launch/receiver_launch.py)**: Starts the receiver with the given parameters.
  * Arguments:
    * **params**: Path to ROS YAML parameter file to load for receiver. If not provided, default
      parameters from this package are loaded.
