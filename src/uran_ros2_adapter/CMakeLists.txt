cmake_minimum_required(VERSION 3.16)
project(uran_ros2_adapter VERSION 2.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nlohmann_json 3.10 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(uran_core_framework 2.1 REQUIRED CONFIG)
find_package(uran_interfaces 1 REQUIRED CONFIG)
find_package(uran_msgs REQUIRED)
find_package(uran_platform_native 2.1 REQUIRED CONFIG)
find_package(uran_plugin_api 2.1 REQUIRED CONFIG)
find_package(uran_ros2_msgs REQUIRED)
find_package(uran_srvs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/conversions.cpp
  src/pluginlib_loader.cpp
  src/ros2_platform.cpp
  src/runtime_host.cpp)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}
  uran::uran_core_framework
  uran::uran_platform_native
  uran::uran_plugin_api
  uran_interfaces::uran_interfaces
  nlohmann_json::nlohmann_json)
ament_target_dependencies(${PROJECT_NAME}
  lifecycle_msgs pluginlib rclcpp rclcpp_lifecycle std_msgs std_srvs
  uran_msgs uran_ros2_msgs uran_srvs)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION 2
  INSTALL_RPATH "$ORIGIN")

# The framework and contract libraries are independently installed packages.
# Keep their discovered prefix in the build RPATH so colcon/CTest can execute
# consumers before an overlay shell has exported every native library path.
set(_uran_native_rpath
  "$<TARGET_FILE_DIR:uran::uran_core_framework>"
  "$<TARGET_FILE_DIR:uran::uran_platform_native>"
  "$<TARGET_FILE_DIR:uran::uran_plugin_api>"
  "$<TARGET_FILE_DIR:uran_interfaces::uran_interfaces>")
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY BUILD_RPATH "${_uran_native_rpath}")

add_executable(uran_runtime_node src/main.cpp)
target_compile_features(uran_runtime_node PRIVATE cxx_std_17)
target_link_libraries(uran_runtime_node ${PROJECT_NAME})
ament_target_dependencies(uran_runtime_node lifecycle_msgs rclcpp)
set_target_properties(uran_runtime_node PROPERTIES INSTALL_RPATH "$ORIGIN/..")
set_property(TARGET uran_runtime_node APPEND PROPERTY BUILD_RPATH "${_uran_native_rpath}")

install(TARGETS ${PROJECT_NAME} EXPORT uran_ros2_adapterTargets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS uran_runtime_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(uran_ros2_adapterTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  lifecycle_msgs nlohmann_json pluginlib rclcpp rclcpp_lifecycle std_msgs std_srvs
  uran_core_framework uran_interfaces uran_msgs uran_platform_native uran_plugin_api
  uran_ros2_msgs uran_srvs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_link_libraries(test_conversions ${PROJECT_NAME})
  set_property(TARGET test_conversions APPEND PROPERTY BUILD_RPATH "${_uran_native_rpath}")
endif()

ament_package()
