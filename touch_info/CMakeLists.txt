cmake_minimum_required(VERSION 3.8)
project(touch_info)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(touch_info REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces( ${PROJECT_NAME} "msg/Touch.msg" DEPENDENCIES geometry_msgs )

include_directories(/opt/OpenHaptics/Developer/Include)
#link_directories(/opt/OpenHaptics/Developer/Lib64)

include_directories( include )
add_library( touch_pub SHARED src/touch_pub.cpp )
ament_target_dependencies( touch_pub rclcpp geometry_msgs touch_info)
target_link_libraries(touch_pub HD)

add_executable( pub src/touch_pub.cpp)
target_link_libraries( pub touch_pub HD)
ament_target_dependencies(pub rclcpp geometry_msgs touch_info)

install( DIRECTORY include/ DESTINATION include)
install( TARGETS touch_pub LIBRARY DESTINATION lib)
install( TARGETS pub DESTINATION lib/${PROJECT_NAME} )
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
