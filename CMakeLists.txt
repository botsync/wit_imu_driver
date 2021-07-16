cmake_minimum_required(VERSION 3.5)
project(wit_imu_driver)
add_compile_options(-std=c++14)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib src/wt901c.cpp)
#ament_target_dependencies(${PROJECT_NAME}_lib rclcpp)
target_link_libraries(${PROJECT_NAME}_lib ${rclcpp_LIBRARIES})

add_executable(wit_imu_driver src/wit_imu_driver_node.cpp)
ament_target_dependencies(wit_imu_driver rclcpp sensor_msgs std_srvs tf2 geometry_msgs ${Boost_EXPORTED_TARGETS})
target_link_libraries(wit_imu_driver
  ${Boost_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${PROJECT_NAME}_lib
)

install(TARGETS wit_imu_driver
  	DESTINATION lib/${PROJECT_NAME}
 )

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

set(ROSLINT_CPP_OPTS "--filter=+,-build/c++14")

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}_lib
)
ament_export_dependencies(
  rclcpp
)
ament_package()
