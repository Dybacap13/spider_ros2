cmake_minimum_required(VERSION 3.8)
project(spider_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    rclcpp_components
    std_msgs
    sensor_msgs
    pluginlib
    )

foreach (Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
  endforeach ()    


include_directories(include)   

add_library(${PROJECT_NAME} SHARED src/controllers_gazebo.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION
                                                 "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})


rclcpp_components_register_node(${PROJECT_NAME}
                                PLUGIN "spider_gazebo_controllers::GazeboControllers"
                                EXECUTABLE spider_gazebo_controllers
                              )
                                
# install
install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

install(EXPORT export_${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

# export
ament_export_include_directories(include)
ament_export_libraries( ${PROJECT_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()