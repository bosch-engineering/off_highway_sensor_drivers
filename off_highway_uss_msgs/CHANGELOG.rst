^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package off_highway_uss_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.0 (2025-03-24)
------------------
* `rosidl_default_generators` is technically a `<buildtool_depend>` (#13)
  rosidl_default_generators is technically a buildtool_depend
* Contributors: Tim Clephas

0.7.0 (2024-12-04)
------------------
* Add object type SNA and filter such objects
* Contributors: Robin Petereit

0.6.3 (2024-07-09)
------------------

0.6.2 (2024-06-13)
------------------

0.6.1 (2024-06-04)
------------------

0.6.0 (2024-05-14)
------------------

0.5.1 (2024-03-27)
------------------
* Disable irrelevant warnings for Clang
  Anonymous structs are used in PCL for type punning:
  https://github.com/PointCloudLibrary/pcl/issues/2303
  Subobject braces are not needed:
  https://bugs.llvm.org/show_bug.cgi?id=21629
  https://gcc.gnu.org/bugzilla/show_bug.cgi?id=25137
* Contributors: Robin Petereit
