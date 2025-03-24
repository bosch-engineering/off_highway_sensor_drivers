^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package off_highway_can
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
