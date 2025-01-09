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

set (SPIDER_CLIENT_LIBRARY_CPP
  src/spider_client_imitation/spider_client_imitation.cpp
  src/spider_ik.cpp
)


#
# SPIDER_CLIENT_LIBRARY_CPP
#

add_library(${PROJECT_NAME} SHARED ${SPIDER_CLIENT_LIBRARY_CPP})
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

ament_export_targets(${PROJECT_NAME}_export HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()   


# #
# # SPIDER_IMITATOR_CPP
# #

# add_library(${PROJECT_NAME}_imitator SHARED ${SPIDER_IMITATOR_CPP})
# #target_link_libraries(${PROJECT_NAME}_imitator )
# set_target_properties(${PROJECT_NAME}_imitator PROPERTIES VERSION
#                                                  "${${PROJECT_NAME}_VERSION}")
# ament_target_dependencies(${PROJECT_NAME}_imitator ${THIS_PACKAGE_INCLUDE_DEPENDS})

# # target_include_directories(
# #   ${PROJECT_NAME}_imitation
# #   PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# #          $<INSTALL_INTERFACE:include>)
# # target_compile_options(${PROJECT_NAME}_imitation PUBLIC ${CXX17_FLAG})

# # install
# install(DIRECTORY include/ DESTINATION include)

# install(
#   TARGETS  ${PROJECT_NAME}_ik 
#   EXPORT export_${PROJECT_NAME}_ik  
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin )

# install(
#     TARGETS ${PROJECT_NAME}_imitator
#     EXPORT  export_${PROJECT_NAME}_imitator
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION bin )

# install(EXPORT export_${PROJECT_NAME}_ik DESTINATION lib/${PROJECT_NAME})
# install(EXPORT export_${PROJECT_NAME}_imitator DESTINATION lib/${PROJECT_NAME})
# # export

# ament_export_targets(export_${PROJECT_NAME}_ik  export_${PROJECT_NAME}_imitator HAS_LIBRARY_TARGET)
# ament_export_libraries( ${PROJECT_NAME}_ik ${PROJECT_NAME}_imitator)
# ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
# ament_export_include_directories(include)


# ament_package()
