cmake_minimum_required(VERSION 3.8)
project(spider_client_library)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    Eigen3
    eigen3_cmake_module
    )


foreach (Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
  endforeach ()    

include_directories(include)  


add_library(${PROJECT_NAME} SHARED src/spider_ik.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION
                                                 "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
                          
# install
install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin )

install(EXPORT export_${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

# export

ament_export_libraries( ${PROJECT_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
#ament_export_include_directories(include)


ament_package()
