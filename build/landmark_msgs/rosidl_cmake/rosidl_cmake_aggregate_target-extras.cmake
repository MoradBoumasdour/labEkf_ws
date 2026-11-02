# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target landmark_msgs::landmark_msgs
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${landmark_msgs_TARGETS}.
if(landmark_msgs_TARGETS AND NOT TARGET landmark_msgs::landmark_msgs)
  add_library(landmark_msgs::landmark_msgs INTERFACE IMPORTED)
  set_target_properties(landmark_msgs::landmark_msgs PROPERTIES
    INTERFACE_LINK_LIBRARIES "${landmark_msgs_TARGETS}")
endif()
