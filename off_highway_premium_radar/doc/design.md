# Design

The driver node is written modular and extendable as shown in the following simplified UML class
diagram:

![Simplified UML class diagram](media/driver_classes.drawio.svg "Simplified UML class diagram")

The `Node` manager class implements the `rclcpp::Node` interface and contains a list of [`Converter`]
instances and an instance of the `Driver` class. The `Driver` class is responsible to set up a
thread via an `IoContext` instance to receive data asynchronously. It also provides a [`Sender`]
interface to send sensor PDUs. The `Driver` class will check received PDUs, deserialize them and
dispatch the resulting PDU structures to a list of [`Receiver`] instances.

For receiving and sending UDP data the `Driver` class uses an instance of the `UdpSocket` class
which handles sending and receiving bytes over the wire. The `UdpSocket` class is based on the
[ASIO](https://think-async.com/Asio/) library and uses an `udp::socket` instance.

On the sending side, the `Driver` class implements a [`Sender`] interface, which allows any user of
that interface to send PDUs to the sensor. The `Driver` will serialize PDU input structures and send
the byte stream via the `UdpSocket` instance. The `Node` class forwards the [`Sender`] interface and
a weak reference to its `rclcpp::Node` interface to all of its [`Converter`] instances.

Each [`Converter`] implements the
[`Receiver`] interface, such that the
`Node` class can register each [`Converter`] as [`Receiver`] in the `Driver` instance.

In conclusion, the list of [`Converter`] instances passed to the `Node` can receive PDUs through the
[`Receiver`] interface, called by the receiving thread in the `IoContext` and can send PDUs through
the [`Sender`] interface to the `Driver` in the main thread.

Thus, the list of [`Converter`] instances is a plugin list, which can be extended by subsequent
packages.

The implemented `DefaultConstructor` maps the received PDU structures into ROS messages through
publishers and on the other hand fills PDU structures from a ROS message subscription / service and
sends them via the [`Sender`] to the sensor. Furthermore, it implements diagnosis for all subscribed
and published topics if they are expected to transmit data cyclically.

[`Receiver`]:
../include/off_highway_premium_radar/interface/receiver.hpp
[`Sender`]:
../include/off_highway_premium_radar/interface/sender.hpp
[`Converter`]:
../include/off_highway_premium_radar/interface/converter.hpp
