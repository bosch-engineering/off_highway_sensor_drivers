^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package off_highway_premium_radar_sample
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.0 (2025-03-24)
------------------

0.7.0 (2024-12-04)
------------------
* Do not force warnings as errors to fix rolling (#11)
* Contributors: Tim Clephas

0.6.3 (2024-07-09)
------------------
* Add missing dependency on PCL headers (#9)
  When I try to build this package on jazzy with rosdep, the PCL headers
  are missing. This package uses `find_package(PCL)` and includes it in
  the headers. Therefore I think it would be best to add it to the
  `build_depend` and `build_export_depend` tags of the `package.xml`
  files.
  In the buildfarm this is technically not needed because the PCL headers
  are a `build_export_depend` of `pcl_conversions`, but rosdep ignores
  this dependency so it misses the PCL headers.
* Contributors: Ramon Wijnands

0.6.2 (2024-06-13)
------------------
* Fix azimuth and elevation angle variances of premium radar (#8)
* Contributors: Maximilian Hilger

0.6.1 (2024-06-04)
------------------

0.6.0 (2024-05-14)
------------------
* Rename premium radar driver to sample
  Currently, the delivered Radar Off-Highway Premium and its firmware are just samples.
  Separate package will be created for series firmware.
* Switch from deprecated signature to const shared_ptr reference while still supporting efficient intra-process communication
  See https://github.com/ros2/rclcpp/pull/1598.
* Interpret azimuth and elevation angle of premium radar locations as cone coordinates
* Update executable names for premium radar in launch
* Convert premium radar from node to component
* Remove redundant normal in pcl
* Contributors: Robin Petereit, Sarah Huber

0.5.1 (2024-03-27)
------------------
* Disable irrelevant warnings for Clang
  Anonymous structs are used in PCL for type punning:
  https://github.com/PointCloudLibrary/pcl/issues/2303
  Subobject braces are not needed:
  https://bugs.llvm.org/show_bug.cgi?id=21629
  https://gcc.gnu.org/bugzilla/show_bug.cgi?id=25137
* Remove unnecessary self assignment
* Clarify operation order
* Remove C style cast
* Use explicit initialization
* Remove unused variables
* Add virtual destructor for base class
* Contributors: Robin Petereit
