cmake_minimum_required(VERSION 3.8)
project(mid360_preprocess)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(mid360_preprocess_node src/mid360_preprocess_node.cpp)

ament_target_dependencies(mid360_preprocess_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(mid360_preprocess_node
  ${PCL_LIBRARIES}
)

install(TARGETS mid360_preprocess_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
