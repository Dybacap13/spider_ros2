cmake_minimum_required(VERSION 3.8)
project(spider_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    spider_client_library   
    controller_manager
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    rclcpp_components
    spider_msgs
    )


foreach (Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
  endforeach ()    

include_directories(include)  


set(HW_ROBOT_CONTROL_CPP src/spider_hardware_interface.cpp)
set(ROBOT_CONTROL_CPP src/spider_control.cpp)
#
# control
#
add_library(spider_control SHARED ${ROBOT_CONTROL_CPP})
set_target_properties(spider_control PROPERTIES VERSION
                                                 "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(spider_control ${THIS_PACKAGE_INCLUDE_DEPENDS})

rclcpp_components_register_nodes(spider_control
                                 "spider_driver::spider_control")

#
# spider_hardware_interface
#

add_library(spider_driver SHARED ${HW_ROBOT_CONTROL_CPP})

set_target_properties(spider_driver PROPERTIES VERSION
                                                   "${${PROJECT_NAME}_VERSION}")
                                                                   
ament_target_dependencies(spider_driver ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(spider_driver spider_control)
rclcpp_components_register_nodes(spider_driver
                                 "spider_driver::spider_driver")

target_compile_definitions(spider_driver
                           PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(hardware_interface
                                         hardware_interface_plugin.xml)

# install
install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS spider_driver spider_control
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)
install(EXPORT export_${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

# export
ament_export_include_directories(include)
ament_export_libraries(spider_driver spider_control)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()