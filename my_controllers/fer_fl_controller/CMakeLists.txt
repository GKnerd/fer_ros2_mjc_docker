cmake_minimum_required(VERSION 3.8)
project(fer_fl_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)

add_executable(feedback_linearization_node src/feedback_linearization_node.cpp)
add_executable(mpc_controller_node src/mpc_controller_node.cpp)

foreach(target feedback_linearization_node mpc_controller_node)
  ament_target_dependencies(${target}
    rclcpp
    sensor_msgs
    std_msgs
    kdl_parser
    orocos_kdl_vendor
  )
  target_link_libraries(${target}
    orocos-kdl
  )
endforeach()

install(TARGETS
  feedback_linearization_node
  mpc_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()