# off_highway_mm7p10

The off_highway_mm7p10 package provides a receiver node to receive and decode CAN frames from the Bosch
MM7.10 Inertial Measurement Unit (IMU) into ROS messages - it implements an `off_highway_can::Receiver`.

Further information on the Bosch MM7.10 IMU, its inputs, outputs and how they can be
interpreted can be found on the [Bosch Rexroth Homepage](https://www.boschrexroth.com).

## Supported devices

| **Device name** | **Part Number** | **Description** |
| -| - | - |
| MM7.10 IMU | R 917 013 362 | - 6-axis IMU (3-axis gyroscope, 3-axis accelerometer)<br> - IMU data output on automotive CAN is supported |

Contact: [**off-highway.beg@bosch.com**](mailto:off-highway.beg@bosch.com?subject=off_highway_sensor_drivers%20MM7P10%20IMU)

## Nodes

### Receiver

The MM7.10 receiver decodes CAN frames into IMU data and publishes it as ROS sensor messages.

The MM7.10 sends its data in three synchronized CAN frames (TX1, TX2, TX3) containing:

- **TX1 (Z_AY)**: Yaw rate, lateral acceleration (Y), temperature, status bits
- **TX2 (X_AX)**: Roll rate, longitudinal acceleration (X), status bits
- **TX3 (Y_AZ)**: Pitch rate, vertical acceleration (Z), hardware index, status bits

All three frames must be received with matching message counters before the data is published. If
frames are missed or message counters don't match, a warning is logged and the data is not published.

The receiver validates message counters across the three frames to ensure synchronization. Messages
with all-zero IMU values are filtered out as invalid.

The IMU data is published as a standard `sensor_msgs/Imu` message. The orientation field is not
populated (covariance set to -1), as the MM7.10 only provides angular velocities and linear
accelerations, not orientation estimates.

Additionally, diagnostic information including sensor status bits and temperature is published as an
`Information` message and monitored for errors. If any status bits indicate an error (IMU not
available, signal failure, or initialization in progress) or if the temperature is invalid, a
diagnostic error is published on `/diagnostics`.

If the receiver node does not receive any CAN messages within the configured timeout period, the watchdog will detect this
and publish a diagnostic error on `/diagnostics`. The watchdog checks for timeouts at the frequency
specified by `watchdog_frequency` (default 100 Hz).

#### Subscribed Topics

* **from_can_bus
  ([`can_msgs/Frame`](http://docs.ros.org/en/noetic/api/can_msgs/html/msg/Frame.html))**
  * MM7.10 CAN frames to decode

#### Published Topics

* **imu
  ([`sensor_msgs/Imu`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html))**
  * Update Rate: On each complete set of synchronized CAN frames (typically 100 Hz), but only if IMU data is not all zeros
  * Contains angular velocity (rad/s) and linear acceleration (m/s²) data from the MM7.10
  * Orientation field is not used (covariance set to -1)
  * Roll rate sign is inverted to match ROS coordinate system conventions
* **info
  ([`off_highway_mm7p10_msgs/Information`](../off_highway_mm7p10_msgs/msg/Information.msg))**
  * Update Rate: On each complete set of synchronized CAN frames (when IMU is published)
  * Contains current sensor status information including status bits for all channels (yaw, roll,
    pitch rates and accelerations) and temperature data
* **/diagnostics
  ([`diagnostic_msgs/DiagnosticArray`](http://docs.ros.org/en/noetic/api/diagnostic_msgs/html/msg/DiagnosticArray.html))**
  * Update Rate: On each complete set of synchronized CAN frames or when watchdog detects timeout
  * Diagnostic status contains:
    * Timeout status (monitored by watchdog timer at `watchdog_frequency`)
    * Sensor status bits from all six channels (yaw, roll, pitch rates and X, Y, Z accelerations)
    * Temperature validity status

#### Parameters

See [receiver_params.yaml](config/receiver_params.yaml).

## Launch files

* **[receiver_launch](launch/receiver_launch.py)**: Starts the receiver with the given parameters.
  * Arguments:
    * **params**: Path to ROS YAML parameter file to load for receiver. If not provided, default
      parameters from this package are loaded.
