^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package off_highway_premium_radar
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.13.0 (2026-11-02)
-------------------
* Add extra validity check for locations

    An extra validation check was added to verify if the received location is a measured one or padding.

    Adapted test cases for new condition.
* Remove io_context dependency

    Due to build issues caused by another package, it was decided to remove the dependency to it in order to have a successful build on rolling.
* Increase accepted timestamp delay for diagnostics to 150 ms

    Larger delays can occur once PTP synchronization is active, so the
    previous 100 ms limit was too strict for off_highway_premium_radar
    and off_highway_premium_radar_sample default converters.
* asio::ip::{address::from_string -> make_address} (#33)

    fix for asio 1.36

    ``from_string`` has been deprecated for at least 9 years (ref blame https://github.com/boostorg/asio/blame/b60e92b13ef68dfbb9af180d76eae41d22e19356/include/boost/asio/ip/address.hpp),
    and removed in https://github.com/boostorg/asio/commit/c0d1cfce7767599c4cf00df36f8017a1073339ae

    Applied to off_highway_premium_radar and off_highway_premium_radar_sample.
* Contributors: Darius Man, David Mueller, Guilhem Saurel

0.12.0 (2026-03-10)
-------------------
* Update maintainers
* Adapt README file
* Contributors: Gabriela Adriana Lapuste, Robin Petereit

0.11.0 (2026-01-26)
-------------------

0.10.0 (2025-11-12)
-------------------
* Remove PCL libraries from ohw_premium_radar
* Add PCL deprecation warning for pcl_radar_point_type.hpp
* Remove ament deprecation warning from remaining packages
* Update README
* Add unit tests for Measurement Program PDU
* Add unit tests for Ego Vehicle Input PDU
* Add unit tests for Location Atrributes PDU
* Add unit test for Location Data PDU
* Contributors: Calin-Vasile Sopterean, Gabriela Adriana Lapuste

0.9.0 (2025-06-16)
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

    Add sensor coating information (blindness)
    Add additional misalignment indicator
    Strip down sensor state information
* Add partial unit tests for sensor state information and blindness indicators
* Add premium radar driver for series sensor
* Contributors: Gabriela Adriana Lapuste, Robin Petereit, Sarah Huber
