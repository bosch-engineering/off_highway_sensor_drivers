^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package off_highway_can
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-11-12)
------------------
* Fix formatting for linter
* off_highway_can: split license element into two
  Having two license strings into one element does give issues when generating
  the Yocto/OpenEmbedde bitbake recipes for meta-ros.
  see: https://github.com/ros/meta-ros/commit/72dfb21b8f297d23e8c029edc04baadb17b1d24f
  see:
  <license> (multiple, but at least one)
  Name of license for this package, e.g. BSD, GPL, LGPL.
  In order to assist machine readability, only include the license name in this tag.
  For multiple licenses multiple separate tags must be used.
  A package will have multiple licenses if different source files have different licenses.
  Every license occurring in the source files should have a corresponding <license> tag.
  For any explanatory text about licensing caveats, please use the <description> tag.
  -- https://ros.org/reps/rep-0149.html#license-multiple-but-at-least-one
* Add unit tests for timeout
* Fix incorrect timeout toggling
* Draw conclusion in the watchdog, not in the diagnostics callback
* Fix issue 20, don't publish sensor data when there is a sensor timeout
* ament_target_dependencies is deprecated for Kilted and Rolling
  This syntax should work as of ROS 2 Foxy already
* Contributors: Calin-Vasile Sopterean, Ferry Schoenmakers, Jan Vermaete, Tim Clephas

1.0.0 (2025-07-01)
------------------
* Use lround and explicit cast
* Fix integer inputs for round test
* Contributors: Robin Petereit

0.8.0 (2025-03-24)
------------------
* Move test to emphasize external FOSS
* Add tests for encoding by round
* Replace truncation of physical value with round outside of FOSS
  Minimizes the floating point error such that half
  offset of resolution in tests is not needed
  anymore.
* Contributors: Calin-Vasile Sopterean, Robin Petereit

0.7.0 (2024-12-04)
------------------
* Update docs for CAN FD
* Check matching lengths in decoding
* Allow start bits in CAN FD range
* Make CRC and message counter optional
* Enable processing of CAN FD frames
* Remove unused definitions
* Do not force warnings as errors to fix rolling (#11)
* Contributors: Robin Petereit, Tim Clephas

0.6.3 (2024-07-09)
------------------

0.6.2 (2024-06-13)
------------------

0.6.1 (2024-06-04)
------------------
* Fix missing dependency that was find_packaged (#7)
  * Fix missing dependency that was find_packaged
  * Use ament_lint_common
  ---------
* Contributors: Tim Clephas

0.6.0 (2024-05-14)
------------------
* Switch from deprecated signature to const shared_ptr reference while still supporting efficient intra-process communication
  See https://github.com/ros2/rclcpp/pull/1598.
* Contributors: Sarah Huber

0.5.1 (2024-03-27)
------------------
* Align Clang FP contraction to GCC for passing tests
* Add virtual destructor for base class
* Contributors: Robin Petereit
