cmake_minimum_required(VERSION 3.5)
project(latency_analysis)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
  rclcpp
  message_filters
  sensor_msgs
)

add_executable(latest_model_test src/latest_model_test.cpp)
target_include_directories(latest_model_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(latest_model_test
  "rclcpp"
  "message_filters"
  "sensor_msgs"
 )

add_executable(modify_improve_test src/modify_improve_test.cpp)
target_include_directories(modify_improve_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(modify_improve_test
  "rclcpp"
  "message_filters"
  "sensor_msgs"
 )

add_executable(latest_evaluation src/latest_evaluation.cpp)
target_include_directories(latest_evaluation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(latest_evaluation
  "rclcpp"
  "message_filters"
  "sensor_msgs"
 )

install(TARGETS 
  latest_model_test
  modify_improve_test
  latest_evaluation
  DESTINATION lib/${PROJECT_NAME})


# Install files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
