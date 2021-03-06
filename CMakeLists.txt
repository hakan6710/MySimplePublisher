cmake_minimum_required(VERSION 3.8)
project(test_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(adp_core_msgs REQUIRED)

find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED )
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED )
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
        include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${class_loader_INCLUDE_DIRS}
  ${rmw_implementation_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${adp_core_msg_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}

)


add_executable(my_publisher src/publisher_node.cpp)
ament_target_dependencies(my_publisher 
cv_bridge
"rclcpp" 
"std_msgs" 
"adp_core_msgs"
"class_loader"
"rclcpp_components"
"sensor_msgs"
"OpenCV"
)





target_link_libraries(my_publisher
  ${rclcpp_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ament_index_cpp_LIBRARIES}
  ${adp_core_msgs}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBS}
  ${cv_bridge_LIBRARIES}
)


# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()



install(TARGETS my_publisher DESTINATION lib/${PROJECT_NAME} )

install(
    TARGETS my_publisher
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)



##MY PBUSLIERH COP
add_executable(my_publisher2 src/publisher_node_copy.cpp)
ament_target_dependencies(my_publisher2 
cv_bridge
"rclcpp" 
"std_msgs" 
"adp_core_msgs"
"class_loader"
"rclcpp_components"
"sensor_msgs"
"OpenCV"
)





target_link_libraries(my_publisher2
  ${rclcpp_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ament_index_cpp_LIBRARIES}
  ${adp_core_msgs}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBS}
  ${cv_bridge_LIBRARIES}
)





install(TARGETS my_publisher2 DESTINATION lib/${PROJECT_NAME} )

install(
    TARGETS my_publisher2
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()