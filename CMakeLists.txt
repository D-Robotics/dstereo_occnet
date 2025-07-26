cmake_minimum_required(VERSION 3.8)
project(dstereo_occnet)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
include_directories(
  ${SYS_ROOT}/usr/include
  ${SYS_ROOT}/usr/hobot/include
)
link_directories(
  ${SYS_ROOT}/usr/lib/hbbpu
  ${SYS_ROOT}/usr/hobot/lib
  ${SYS_ROOT}/usr/lib
)

# add executable
# add_executable(dstereo_occnet
#   src/main.cpp
#   src/dstereo_occnet_node.cpp
#   src/dstereo_occnet_infer.cpp
#   src/bpu_utils.cpp
#   src/img_convert_utils.cpp
#   src/pc_utils.cpp
#   src/timer_utils.cpp
#   src/thread_pool.cpp
#   src/file_utils.cpp
# )
# add libraries
add_library(${PROJECT_NAME}_component SHARED
  src/dstereo_occnet_node.cpp
  src/dstereo_occnet_infer.cpp
  src/bpu_utils.cpp
  src/img_convert_utils.cpp
  src/pc_utils.cpp
  src/timer_utils.cpp
  src/thread_pool.cpp
  src/file_utils.cpp
)
target_compile_features(${PROJECT_NAME}_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  OpenCV
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME}_component
  dnn
)
# compile executable file
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "DStereoOccNetNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

# install
install(TARGETS ${PROJECT_NAME}_component RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
