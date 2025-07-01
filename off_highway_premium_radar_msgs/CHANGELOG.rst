^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package off_highway_premium_radar_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2025-07-01)
------------------
* Remove measurement cycle synchronization PDU
  Measurement cycle synchronization will be handled via diagnosis in series sensor.
* Remove sensor DTC  information PDU
* Remove sensor feedback PDU
* Remove sensor mode request PDU
* Switch to custom message for sensor mounting position
  This removes the dependency to tf2.
* Remove Sensor Broadcast message
* Implement LGP interface changes for series sensor
  - Add sensor coating information (blindness)
  - Add additional misalignment indicator
  - Strip down sensor state information
* Add premium radar driver for series sensor
* Contributors: Sarah Huber
