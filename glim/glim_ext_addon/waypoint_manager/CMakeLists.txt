cmake_minimum_required(VERSION 3.22)
project(waypoint_manager)

set(CMAKE_CXX_STANDARD 17)

find_package(glim REQUIRED)
find_package(GTSAM REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)

if($ENV{ROS_VERSION} EQUAL 2)
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  ament_auto_add_library(waypoint_manager SHARED
    src/glim_ext/waypoint_manager_module_ros2.cpp
  )
endif()

target_include_directories(waypoint_manager PRIVATE
  include
  ${GTSAM_INCLUDE_DIRS}
  ${glim_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)
target_link_libraries(waypoint_manager
  glim_ext
  ${GTSAM_LIBRARIES}
  ${glim_LIBRARIES}
  spdlog::spdlog
  yaml-cpp
)
