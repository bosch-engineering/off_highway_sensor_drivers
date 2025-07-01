^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package off_highway_premium_radar
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2025-07-01)
------------------
* Adapt expected frequencies of diagnostic messages
* Adapt documentation of sensor state information
* Adapt documentation of measurement program
* Remove measurement cycle synchronization PDU
  Measurement cycle synchronization will be handled via diagnosis in series sensor.
* Remove sensor DTC  information PDU
* Remove sensor feedback PDU
* Remove sensor mode request PDU
* Update README
* Disable publishing of diagnosis for sensor feedback PDU
  PDU is not implemented yet in sensor software but will be available again in a future release.
* Fix review findings
* Adapt diagnostic messages
* Switch to custom message for sensor mounting position
  This removes the dependency to tf2.
* Remove Sensor Broadcast message
* Remove signal range checks as this should be done in sensor and / or application software
* Set IP addresses in default configuration to default values of series sensor
* Replace PCL point struct with message modifier and iterator
* Implement LGP interface changes for series sensor
  - Add sensor coating information (blindness)
  - Add additional misalignment indicator
  - Strip down sensor state information
* Add partial unit tests for sensor state information and blindness indicators
* Add premium radar driver for series sensor
* Contributors: Gabriela Adriana Lapuste, Robin Petereit, Sarah Huber
