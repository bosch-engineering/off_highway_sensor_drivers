# Intended use

The intended use of the software as well as the sensors is for mobile machines and mobile robotics
according to the [machinery directive
2006/42/EC](https://eur-lex.europa.eu/legal-content/EN/TXT/?uri=celex%3A32006L0042) only. The
provided *off_highway_sensor_drivers* software is a prototype software for surround object detection
for use on mobile machines (non-road mobile machinery).

1. Scope of intended use
    1. The *off_highway_sensor_drivers* software is a prototype software, which can be used for an
       assistance systems, object visualization or collecting sensor data.
    2. Definitions: The user/operator/driver are persons, which operate/use the
       *off_highway_sensor_drivers* or the machine whereon the software/system is installed.
    3. The *off_highway_sensor_drivers* are in prototype status and the intended use is limited to
       mobile machine demonstrators according to the machinery directive.
    4. The operation of the system is limited to selected test areas on the user's premises.
    5. The prototype system is designed as an assistance system and does not autonomously intervene
       on any machine control (e.g. brake or steering).
    6. The responsibility for road and machine safety remains with the driver/operator of the mobile
       machine.
    7. The intended use of the single system components is still valid and is described in the
       related technical documentation.
2. Performance note
    1. The published object information of the node may deviate from the real performance of the
       sensors. Reasons can be the settings of the CAN bus, delays in data processing, incorrect
       configuration or similar aspects.
    2. In general, the object detection performance of a system is limited to the detection
       performance of the individual sensor technologies described in the respective technical
       documentation. These limitations are based on physical or technical basic prerequisites of
       the components, their installation and configuration related effects, not this software
       library.
3. Users
    1. The product is designed for the field of application described above exclusively and is only
       to be used by qualified and instructed persons.
