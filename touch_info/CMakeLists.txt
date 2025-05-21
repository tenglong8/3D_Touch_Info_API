cmake_minimum_required(VERSION 3.8)
project(touch_info)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- Generate message ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Touch.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# --- Include dirs (add generated message headers) ---
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# --- Link external headers/libs if needed ---
include_directories(/opt/OpenHaptics/Developer/Include)
link_directories(/opt/OpenHaptics/Developer/Lib64)

# --- Build library (optional) ---
add_library(touch_pub SHARED src/touch_pub.cpp)
ament_target_dependencies(touch_pub rclcpp geometry_msgs rosidl_default_runtime)
target_link_libraries(touch_pub HD)
#add_dependencies(touch_pub ${PROJECT_NAME}__rosidl_generator_cpp)  # Depends on generated msgs

# --- Build executable ---
add_executable(pub src/touch_pub.cpp)
target_link_libraries(pub touch_pub HD)
ament_target_dependencies(pub rclcpp geometry_msgs rosidl_default_runtime)
#add_dependencies(pub ${PROJECT_NAME}__rosidl_generator_cpp)


rosidl_target_interfaces(touch_pub
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(pub
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# --- Install ---
install(DIRECTORY include/ DESTINATION include)
install(TARGETS touch_pub LIBRARY DESTINATION lib)
install(TARGETS pub DESTINATION lib/${PROJECT_NAME})

# --- Testing (optional) ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

