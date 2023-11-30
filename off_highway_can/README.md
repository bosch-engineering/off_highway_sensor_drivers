# off_highway_can

The off_highway_can package provides a C++ library to

* receive `can_msgs/Frame` ROS messages and decode their bytes into custom C++ message structures,
* encode the bytes of `can_msgs/Frame` ROS messages from custom C++ message structures and send
  them.

The list of message structures containing the definition of a CAN message and signals need to be
filled by derived classes for CAN node specific functionality. The more generic handling of a cyclic
redundancy check and a rolling message counter per message is provided by the library. Have a look
directly into the [CAN message structure definition](include/off_highway_can/can_message.hpp) for
further information.

## Classes

### Receiver

The `off_highway_can::Receiver` is an abstract base class and needs to be derived for specific
sensor types and behavior. It provides the functionality to process
[`can_msgs/Frame`](http://docs.ros.org/en/noetic/api/can_msgs/html/msg/Frame.html) ROS messages and
decode configured ones based on provided message definitions. The decoded signal values are
forwarded to the derived class for user-defined processing of them.

The processing chain is based completely on message callbacks and thus event-based. The needed
processing time per message can be logged as ROS debug message at runtime with the compile time
option[`COMPILE_DEBUG_LOG`](CMakeLists.txt).

Furthermore, the `off_highway_can::Receiver` provides a watchdog which checks whether any configured
message was received in the period of time defined by the parameter `timeout` and sends a diagnostic
error on the `/diagnostics` topic when a timeout occurs.

#### Subscribed Topics

* **from_can_bus
  ([`can_msgs/Frame`](http://docs.ros.org/en/noetic/api/can_msgs/html/msg/Frame.html))**
  * CAN frames to decode

#### Published Topics

* **/diagnostics
  ([`diagnostic_msgs/DiagnosticArray`](http://docs.ros.org/en/noetic/api/diagnostic_msgs/html/msg/DiagnosticArray.html))**
  * Update Rate: normally 1 Hz, can be forced
  * Diagnostic error in base class only contains timeout status

#### Parameters

See [receiver_params.yaml](config/receiver_params.yaml).

### Sender

The `off_highway_can::Sender` is an abstract base class and needs to be derived for specific
behavior. It provides the functionality to send
[`can_msgs/Frame`](http://docs.ros.org/en/noetic/api/can_msgs/html/msg/Frame.html) ROS messages by
encoding configured CAN messages as bytes based on provided message definitions. The encoded signal
values are published to the `to_can_bus` topic.

The encoding and publishing chain is just a method call and thus event-based. The needed processing
time to encode and publish all configured messages can be logged as ROS debug message at runtime
with the compile time option[`COMPILE_DEBUG_LOG`](CMakeLists.txt).

Furthermore, the `off_highway_can::Sender` provides a watchdog which checks whether a ROS message
was published in a defined by the parameter `timeout` and sends a diagnostic error on the
`/diagnostics` topic when a timeout occurs.

#### Published Topics

* **to_can_bus
  ([`can_msgs/Frame`](http://docs.ros.org/en/noetic/api/can_msgs/html/msg/Frame.html))**
  * Encoded CAN frames
* **/diagnostics
  ([`diagnostic_msgs/DiagnosticArray`](http://docs.ros.org/en/noetic/api/diagnostic_msgs/html/msg/DiagnosticArray.html))**
  * Update Rate: normally 1 Hz, can be forced
  * Diagnostic status in base class only contains timeout status

#### Parameters

See [sender_params.yaml](config/sender_params.yaml).

## Free and Open Source Software (FOSS)

This library uses the [libcan-encode-decode](https://github.com/reinzor/libcan-encode-decode)
library for encoding and decoding raw bytes into / from floating values. See the [FOSS
documentation](foss_documentation/) for further information.
