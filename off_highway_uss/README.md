# off_highway_uss

The off_highway_uss package provides a receiver node to receive and decode CAN frames of the Bosch
Ultrasonic Sensor System (USS) Off-Highway into ROS messages - it implements an
`off_highway_can::Receiver`. Furthermore, the package provides a sender node to encode and send
needed USS input data as CAN frames - an implementation of an `off_highway_can::Sender`.

Further information on the [variants of the Bosch Ultrasonic Sensor System Off-Highway](https://www.bosch-mobility-solutions.com/en/solutions/assistance-systems/ultrasonic-sensor-systems-ohw/) can be found in the related datasheets or the Technical Documentation of the sensor system.

## Supported devices

| **Device name** | **Part Number** | **Description** | **Datasheet** |
| - | - | - | - |
| Ultrasonic Sensor System OHW Entry | - F037.000.145 (series) <br> from software version SW2.0.0 | - Ultrasonic sensor system with up to 12 sensors <br> - Up to 36 distances <br> - Distance output on automotive CAN is supported | [Link](https://www.bosch-mobility.com/media/global/solutions/off-highway-and-large-engines/driver-assistance-systems/ultrasonic-sensor-systems/beg_onepager_uss_en_rgb_20211209.pdf) |
| Ultrasonic Sensor System OHW Premium | - F037.000.125  (series) <br> - F037.B00.672-xx (sample) <br> both from software version SW2.0.0 | - Ultrasonic sensor system with up to 12 sensors <br> - Up to 20 objects and up to 36 distances <br> - Object and distance output on automotive CAN is supported | [Link](https://www.bosch-mobility.com/media/global/solutions/off-highway-and-large-engines/driver-assistance-systems/ultrasonic-sensor-systems/beg_onepager_uss_premium_en_rgb_20211209.pdf) |
| Ultrasonic Sensor System OHW Premium Safe |- F037.B01.117-xx (sample)<br> - F037.000.210 (series) |- Ultrasonic sensor system developed in accordance with ISO 13849 and ISO 25119 up to PL d, AgPL d respectively for the Direct Echo functionality <br> - This driver package is not developed according to safety standards and not suitable for safety applications.| [Link](https://www.bosch-mobility.com/media/global/solutions/off-highway-and-large-engines/driver-assistance-systems/ultrasonic-sensor-systems/onepager_uss_premiumsafe_en.pdf) |

Contact: [**off-highway.beg@bosch.com**](mailto:off-highway.beg@bosch.com?subject=off_highway_sensor_drivers%20Ultrasonic%20OHW%20Sensors)

## Nodes

### Receiver

The USS receiver decodes CAN frames into an object list and a direct echo list, manages the current
lists and publishes them cyclically.

The parameter `use_j1939` defines whether automotive CAN or J1939 messages are expected. J1939
support has the following limitations:

* A fixed source address for the USS ECU must be configured. The dynamic assignment of source
  addresses is not supported.
* The receiver expects the parameter group numbers (PGNs) to be arranged as in the default DBC. A
  rearrangement is only supported as a shift of the entire block. Moving individual PGNs is not
  supported. Be aware that the PGN is only bit 8 to 23 of the extended CAN ID.

All received messages are checked for their cyclic redundancy check (CRC), rolling message counter
and age (message not older than parameter `allowed_age`). If any of these checks do not succeed the
received message is not further processed and skipped.

The relevant USS CAN frame IDs to process (`object_base_id`, `direct_echo_base_id`,
`max_detection_range_id` and `info_id`) are calculated from the `id_offset` (automotive CAN) or
`pgn_offset` (J1939) parameter with constant offsets. It should correspond to the value configured
in the USS ECU and needs to be adapted for the specific bus setup. When using J1939, the receiver
also checks whether the source address contained in the message matches the value of the
`source_address` parameter. If multiple USS systems need to be decoded on the same bus just launch
multiple receiver nodes with individual `id_offset` (automotive CAN) or `pgn_offset` and
`source_address` (J1939) parameters.

The object data is published as a list of objects or as a point cloud and contains up to 20 objects
(sensor limit). Only valid objects in the list are published (type not `TYPE_NONE`). Line objects in
the point cloud format are sampled between their first and second position into individual points
based on the `line_sample_distance` parameter. If this parameter is zero, only the end points of the
line segment are put in the point cloud.

An object CAN frame is multiplexed by the USS, so that the same CAN frame ID is used for two
objects. Thus, for uniqueness, the `id` fields of an object in both published list formats are
computed by adding the multiplexor value times ten to the the enumeration the CAN frame IDs starting
from the `object_base_id` as zero. This results in an object `id` in the range [0, 20).

The direct echo data is published as a list of direct echos and contains up to 12 direct echos
(fills up at start till one set is received). The echo's `id` field is the enumeration of the CAN
frame IDs starting from the `direct_echo_base_id` as zero. This results in an echo `id` in the range
[0, 12).

The echo data list is accompanied by the `max_detection_range` message, which contains the current
maximum detection range for all 12 individual ultrasonic sensors as array.

If the receiver node does not receive anything within a configured period (parameter `timeout`), it
will publish a diagnostic error on `/diagnostics`. Also, the sensor information CAN frame is
published and checked as diagnostic status of the USS. On error, the node publishes a diagnostic
error.

#### Subscribed Topics

* **from_can_bus
  ([`can_msgs/Frame`](http://docs.ros.org/en/noetic/api/can_msgs/html/msg/Frame.html))**
  * USS CAN frames to decode

#### Published Topics

* **objects
  ([`off_highway_uss_msgs/Objects`](../off_highway_uss_msgs/msg/Objects.msg))**
  * Update Rate: configurable with `publish_frequency`
  * Contains current object list of USS as custom message containing the same information as each
    object's CAN frame. Mapping to the respective CAN frame and multiplexor value is done via the
    `id` field.
* **objects_pcl
  ([`sensor_msgs/PointCloud2`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html))**
  * Update Rate: configurable with `publish_frequency`
  * Contains current object list of USS as point cloud. Line objects are sampled with
    `line_sample_distance` to equidistant points along the first and second position. Mapping to the
    respective CAN frame and multiplexor value is done via the `id` field.
* **direct_echos
  ([`off_highway_uss_msgs/DirectEchos`](../off_highway_uss_msgs/msg/DirectEchos.msg))**
  * Update Rate: configurable with `publish_frequency`
  * Contains current direct echo list of USS as custom message containing the same information as
    each echo's CAN frame. Mapping to the respective CAN frame is done via the `id` field.
* **maximum_detection_range
  ([`off_highway_uss_msgs/MaxDetectionRange`](../off_highway_uss_msgs/msg/MaxDetectionRange.msg))**
  * Update Rate: On each sensor maximum detection range CAN frame
  * Contains the current maximum detection range of USS as custom message containing the same
    information as the respective CAN frame.
* **info
  ([`off_highway_uss_msgs/Information`](../off_highway_uss_msgs/msg/Information.msg))**
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

The USS sender listens for ROS ambient temperature messages as input and encodes and publishes these
as CAN frame.

In the case that there is no ambient temperature reading available, the USS can be configured with a
fixed value (see TCD) and this sender node is not needed.

The CAN temperature signal can only encode the range of [-40&deg;C, 87&deg;C], ROS messages
containing temperature values outside of this range are discarded and not encoded / published.
Furthermore, if the ROS temperature message is too old (parameter `allowed_age`) it is discarded,
too.

The relevant USS CAN frame ID to publish the temperature is specified by the
`outside_temperature_id` parameter. This parameter needs to correspond to the vehicle data ID for
the specific bus setup. If multiple USS systems are connected on the same bus, a single USS sender
is sufficient as long as these USS systems expect the vehicle data frame on the same ID (in contrast
to multiple receivers).

If the sender node does not receive anything valid within a configured period (parameter `timeout`),
it will publish a diagnostic error on `/diagnostics`.

#### Subscribed Topics

* **temperature
  ([`sensor_msgs/Temperature`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Temperature.html))**
  * Ambient temperature is sent to the USS for compensating readings

#### Published Topics

* **to_can_bus
  ([`can_msgs/Frame`](http://docs.ros.org/en/noetic/api/can_msgs/html/msg/Frame.html))**
  * Encoded USS CAN frames
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
