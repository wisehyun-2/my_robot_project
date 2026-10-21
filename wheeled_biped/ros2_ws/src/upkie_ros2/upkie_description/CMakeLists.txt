cmake_minimum_required(VERSION 3.8)
project(upkie_description VERSION 2.2.0)

find_package(ament_cmake REQUIRED)

# Thin ament wrapper around the vendored upstream package (upkie/upkie_description is a catkin /
# ROS 1 package).  Its meshes, urdf and xacro directories are installed from the project's models
# folder so that nothing under models/ has to be modified.
set(UPKIE_DESCRIPTION_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../models/upkie_description"
    CACHE PATH "Checkout of upkie_description whose meshes/urdf/xacro directories are installed")
get_filename_component(_upkie_src "${UPKIE_DESCRIPTION_SOURCE_DIR}" ABSOLUTE)
if(NOT EXISTS "${_upkie_src}/xacro/upkie.xacro")
  message(FATAL_ERROR
    "upkie_description sources not found at '${_upkie_src}' "
    "(expected <project>/models/upkie_description); "
    "pass --cmake-args -DUPKIE_DESCRIPTION_SOURCE_DIR=<path> to colcon build")
endif()

install(DIRECTORY "${_upkie_src}/meshes" "${_upkie_src}/urdf" "${_upkie_src}/xacro"
        DESTINATION share/${PROJECT_NAME})

# upkie.xacro includes its parts with paths relative to the *package root* ("xacro/utils/...",
# "xacro/left_leg.xacro"); upstream processes it with xacrodoc from the package root.  ROS xacro
# resolves includes relative to the including file, so a copy of the top-level xacro files at the
# share root makes them processable with
#   xacro $(ros2 pkg prefix --share upkie_description)/upkie.xacro
install(FILES "${_upkie_src}/xacro/upkie.xacro" "${_upkie_src}/xacro/upkie_camera.xacro"
        DESTINATION share/${PROJECT_NAME})

ament_package()
