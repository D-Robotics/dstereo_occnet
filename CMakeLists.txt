cmake_minimum_required(VERSION 3.8)
project(dstereo_occnet)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set default build type (can be overridden by external command)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# compiler optimization settings
set(DNN_LIBS "")
if(PLATFORM_S100)
  message("=> build platform S100")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} \
  -O3 -DNDEBUG \
  -mcpu=cortex-a78 \
  -mtune=cortex-a78 \
  -funroll-loops \
  -finline-functions -finline-limit=1000 \
  -fomit-frame-pointer \
  -fstrict-aliasing \
  -ffp-contract=fast -fno-trapping-math -fno-signaling-nans \
  -frounding-math \
  -flto=auto -fuse-linker-plugin")
  add_definitions(-DPLATFORM_S100)
  set(LAUNCH_DIR "s100")
  list(APPEND DNN_LIBS dnn hbucp)
else()
  message("=> build platform X5")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} \
  -O3 -DNDEBUG \
  -mcpu=cortex-a55 \
  -mtune=cortex-a55 \
  -funroll-loops \
  -finline-functions -finline-limit=800 \
  -fomit-frame-pointer \
  -fstrict-aliasing \
  -ffp-contract=fast -fno-trapping-math -fno-signaling-nans \
  -frounding-math \
  -flto=auto -fuse-linker-plugin")
  add_definitions(-DPLATFORM_X5)
  set(LAUNCH_DIR "x5")
  list(APPEND DNN_LIBS dnn cnn_intf hbrt_bayes_aarch64)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

set(SYS_ROOT ${CMAKE_SYSROOT})
message("SYS_ROOT is " ${SYS_ROOT})
include_directories(
  ${SYS_ROOT}/usr/include
  ${SYS_ROOT}/usr/hobot/include
)
link_directories(
  ${SYS_ROOT}/usr/lib/hbbpu
  ${SYS_ROOT}/usr/hobot/lib
  ${SYS_ROOT}/usr/lib
)

include(FetchContent)
FetchContent_Declare(
  magic_enum
  GIT_REPOSITORY https://github.com/Neargye/magic_enum.git
  GIT_TAG v0.9.7
)
FetchContent_MakeAvailable(magic_enum)

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
target_compile_features(${PROJECT_NAME}_component PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
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
  ${DNN_LIBS}
  magic_enum::magic_enum
)
# compile executable file
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "DStereoOccNetNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

# install
install(TARGETS ${PROJECT_NAME}_component RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
file(GLOB LAUNCH_FILES
  launch/*launch.py
  launch/${LAUNCH_DIR}/*launch.py
)
install(FILES ${LAUNCH_FILES} DESTINATION share/${PROJECT_NAME}/launch)
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
