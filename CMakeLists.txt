cmake_minimum_required(VERSION 3.5)
project(sdkeli_ls_udp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})
endif()

add_executable(sdkeli_ls2027de_node 
  src/sdkeli_ls2027de_node.cpp
  src/sdkeli_ls2027de.cpp
  src/sdkeli_ls1207de_parser.cpp
  src/sdkeli_ls_sensor_frame.cpp
  src/sdkeli_ls_common.cpp
  src/sdkeli_ls_common_udp.cpp
  src/parser_base.cpp
)

target_include_directories(sdkeli_ls2027de_node PRIVATE include)
target_link_libraries(sdkeli_ls2027de_node ${Boost_LIBRARIES})
ament_target_dependencies(sdkeli_ls2027de_node
  rclcpp
  sensor_msgs
  Boost
)


install(TARGETS
sdkeli_ls2027de_node
  DESTINATION lib/${PROJECT_NAME})

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
