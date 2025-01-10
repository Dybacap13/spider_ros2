cmake_minimum_required(VERSION 3.8)
project(spider_imitator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    spider_client_library
    )


foreach (Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
  endforeach ()    

include_directories(include) 


add_library(${PROJECT_NAME} SHARED src/spider_client_imitation.cpp)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)


target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra
                                               -Wno-unused-parameter)

target_compile_options(${PROJECT_NAME} PUBLIC ${CXX17_FLAG})

install(DIRECTORY include/ DESTINATION include/ PATTERN "*private*" EXCLUDE)

install(
  TARGETS ${PROJECT_NAME}  
  EXPORT ${PROJECT_NAME}_export 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


install(EXPORT ${PROJECT_NAME}_export DESTINATION lib/${PROJECT_NAME}) 

ament_export_targets(${PROJECT_NAME}_export  HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()   
