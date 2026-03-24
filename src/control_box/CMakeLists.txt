cmake_minimum_required(VERSION 3.5)
project(control_box)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(example_interfaces REQUIRED)

add_executable(ai1616llpe src/ai1616llpe.cpp)
ament_target_dependencies(ai1616llpe rclcpp std_msgs)

add_executable(ai1616llpe_test src/ai1616llpe_test.cpp)
ament_target_dependencies(ai1616llpe_test rclcpp std_msgs)

add_executable(ao1608llpe src/ao1608llpe.cpp)
ament_target_dependencies(ao1608llpe rclcpp std_msgs example_interfaces)

add_executable(ao1608llpe_test src/ao1608llpe_test.cpp)
ament_target_dependencies(ao1608llpe_test rclcpp std_msgs example_interfaces)

add_executable(cnt3204mtlpe src/cnt3204mtlpe.cpp)
ament_target_dependencies(cnt3204mtlpe rclcpp std_msgs)

add_executable(cnt3204mtlpe_test src/cnt3204mtlpe_test.cpp)
ament_target_dependencies(cnt3204mtlpe_test rclcpp std_msgs)

install(TARGETS
  ai1616llpe
  ai1616llpe_test
  ao1608llpe
  ao1608llpe_test
  cnt3204mtlpe
  cnt3204mtlpe_test
  DESTINATION lib/${PROJECT_NAME}
  )

# # Install launch files.
# install(DIRECTORY
#   launch_test
#   DESTINATION share/${PROJECT_NAME}/
#   )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
