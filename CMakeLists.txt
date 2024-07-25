cmake_minimum_required(VERSION 3.5)
project(vicon_publisher)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(VICON_SDK ViconDataStreamSDK_1.11.0.128037h__Linux64)
set(VICON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/${VICON_SDK})

set(VICON_LIBRARIES
  ${VICON_DIR}/libViconDataStreamSDK_CPP.so
  ${VICON_DIR}/libboost_chrono.so.1.68.0
  ${VICON_DIR}/libboost_system.so.1.68.0
  ${VICON_DIR}/libboost_thread.so.1.68.0
  ${VICON_DIR}/libboost_timer.so.1.68.0
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${VICON_DIR}
)

add_executable(vicon_publisher_node src/vicon/vicon.cpp)

target_link_libraries(vicon_publisher_node
  ${VICON_LIBRARIES}
)

install(DIRECTORY third_party/
  DESTINATION third_party
)

install(FILES
  ${VICON_LIBRARIES}
  DESTINATION lib
)

ament_target_dependencies(vicon_publisher_node rclcpp std_msgs geometry_msgs)

install(TARGETS
  vicon_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

