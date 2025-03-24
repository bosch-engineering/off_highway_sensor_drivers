^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package off_highway_uss
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.0 (2025-03-24)
------------------

0.7.0 (2024-12-04)
------------------
* Add object type SNA and filter such objects
* Do not force warnings as errors to fix rolling (#11)
* Contributors: Robin Petereit, Tim Clephas

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

0.6.1 (2024-06-04)
------------------

0.6.0 (2024-05-14)
------------------
* Switch from deprecated signature to const shared_ptr reference while still supporting efficient intra-process communication
  See https://github.com/ros2/rclcpp/pull/1598.
* Convert USS driver from node to component
* Add launch argument for parameters and unify launch file names
* Contributors: Robin Petereit, Sarah Huber

0.5.1 (2024-03-27)
------------------
* Include what you use
* Remove unused variable
* Properly include rclcpp/executors.hpp when using spin_some
* Disable irrelevant warnings for Clang
  Anonymous structs are used in PCL for type punning:
  https://github.com/PointCloudLibrary/pcl/issues/2303
  Subobject braces are not needed:
  https://bugs.llvm.org/show_bug.cgi?id=21629
  https://gcc.gnu.org/bugzilla/show_bug.cgi?id=25137
* Contributors: Ramon Wijnands, Robin Petereit
