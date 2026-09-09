# Install script for directory: /root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/ros2_bridge/nav3d_ros2_bridge

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/install/nav3d_ros2_bridge")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/build/nav3d_ros2_bridge/nav3d_core_build/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/build/nav3d_ros2_bridge/nav3d_tools_build/cmake_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav3d_ros2_bridge/environment" TYPE FILE FILES "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/build/nav3d_ros2_bridge/ament_cmake_environment_hooks/nav3d_octomap_msgs_library_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav3d_ros2_bridge/environment" TYPE FILE FILES "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/build/nav3d_ros2_bridge/ament_cmake_environment_hooks/nav3d_octomap_msgs_library_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nav3d_ros2_bridge/nav3d_bridge_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nav3d_ros2_bridge/nav3d_bridge_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nav3d_ros2_bridge/nav3d_bridge_node"
         RPATH "/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/nav3d_ros2_bridge" TYPE EXECUTABLE FILES "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/build/nav3d_ros2_bridge/nav3d_bridge_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nav3d_ros2_bridge/nav3d_bridge_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nav3d_ros2_bridge/nav3d_bridge_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nav3d_ros2_bridge/nav3d_bridge_node"
         OLD_RPATH "/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu:"
         NEW_RPATH "/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nav3d_ros2_bridge/nav3d_bridge_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/nav3d_ros2_bridge" TYPE PROGRAM FILES
    "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/ros2_bridge/nav3d_ros2_bridge/scripts/nav3d_cmd_vel_pose_sim.py"
    "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/ros2_bridge/nav3d_ros2_bridge/scripts/nav3d_local_replan_smoke.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav3d_ros2_bridge" TYPE DIRECTORY FILES
    "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/ros2_bridge/nav3d_ros2_bridge/launch"
    "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/ros2_bridge/nav3d_ros2_bridge/rviz"
    "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/ros2_bridge/nav3d_ros2_bridge/config"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/build/nav3d_ros2_bridge/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/nav3d_ros2_bridge")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/build/nav3d_ros2_bridge/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/nav3d_ros2_bridge")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav3d_ros2_bridge/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav3d_ros2_bridge/environment" TYPE FILE FILES "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/build/nav3d_ros2_bridge/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav3d_ros2_bridge/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav3d_ros2_bridge/environment" TYPE FILE FILES "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/build/nav3d_ros2_bridge/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav3d_ros2_bridge" TYPE FILE FILES "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/build/nav3d_ros2_bridge/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav3d_ros2_bridge" TYPE FILE FILES "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/build/nav3d_ros2_bridge/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav3d_ros2_bridge" TYPE FILE FILES "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/build/nav3d_ros2_bridge/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav3d_ros2_bridge" TYPE FILE FILES "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/build/nav3d_ros2_bridge/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav3d_ros2_bridge" TYPE FILE FILES "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/build/nav3d_ros2_bridge/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/build/nav3d_ros2_bridge/ament_cmake_index/share/ament_index/resource_index/packages/nav3d_ros2_bridge")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav3d_ros2_bridge/cmake" TYPE FILE FILES
    "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/build/nav3d_ros2_bridge/ament_cmake_core/nav3d_ros2_bridgeConfig.cmake"
    "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/build/nav3d_ros2_bridge/ament_cmake_core/nav3d_ros2_bridgeConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav3d_ros2_bridge" TYPE FILE FILES "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/ros2_bridge/nav3d_ros2_bridge/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/root/nav/livox_ws/src/3D-Nav-Dynamic-EGO/build/nav3d_ros2_bridge/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
