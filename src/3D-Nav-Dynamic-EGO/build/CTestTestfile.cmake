# CMake generated Testfile for 
# Source directory: /root/nav/livox_ws/src/3D-Nav-Dynamic-EGO
# Build directory: /root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(ros2_bridge_contract "/usr/bin/cmake" "-DPROJECT_SOURCE_DIR=/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO" "-P" "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/tools/check_ros2_bridge_contract.cmake")
set_tests_properties(ros2_bridge_contract PROPERTIES  _BACKTRACE_TRIPLES "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/CMakeLists.txt;12;add_test;/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/CMakeLists.txt;0;")
add_test(plan_status_contract "/usr/bin/cmake" "-DPROJECT_SOURCE_DIR=/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO" "-P" "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/tools/check_plan_status_contract.cmake")
set_tests_properties(plan_status_contract PROPERTIES  _BACKTRACE_TRIPLES "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/CMakeLists.txt;18;add_test;/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/CMakeLists.txt;0;")
subdirs("core")
subdirs("tools")
