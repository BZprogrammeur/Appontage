cmake_minimum_required(VERSION 3.10)
project(wind_plugin)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz-sim7 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-msgs10 REQUIRED)    
find_package(wind_msgs REQUIRED)


add_library(wind_plugin SHARED src/wind_plugin.cpp)

ament_target_dependencies(wind_plugin rclcpp geometry_msgs)
ament_target_dependencies(wind_plugin rclcpp geometry_msgs wind_msgs)    

target_link_libraries(wind_plugin
  gz-sim7::gz-sim7
  gz-plugin2::gz-plugin2
  gz-math7::gz-math7
  gz-msgs10::gz-msgs10                  
)

install(TARGETS wind_plugin
  DESTINATION lib/${PROJECT_NAME})

ament_package()