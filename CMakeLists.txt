cmake_minimum_required(VERSION 3.8)
project(hiro_collision_avoidance_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)

# Include directories
include_directories(
  include
  src/alglib/src
  ${EIGEN3_INCLUDE_DIRS}
)

# Collect alglib sources first
file(GLOB alglib_SRC
    "src/alglib/src/*.h"
    "src/alglib/src/*.cpp"
)

# Collect all source files
# NOTE: You need to add the missing .cpp files or comment out references to them
set(SOURCES
  src/Main.cpp
  src/CartesianPositionController.cpp
  src/KDLSolver.cpp
  src/QPAvoidance.cpp
  src/JointVelocityController.cpp
  src/CollisionAvoidanceBase.cpp
  src/HIROAvoidance.cpp
  src/zScore.cpp
  src/utils/JointLimits.cpp
  # src/FlaccoAvoidance.cpp              # MISSING - optional, for Flacco mode
  # src/HIROCollaborativeController.cpp  # MISSING - optional, for HIROCollaborative mode
  # src/utils/LoggingPublisher.cpp       # Not needed - logging removed
)

# Create executable
add_executable(Main ${SOURCES} ${alglib_SRC})

# Link dependencies
ament_target_dependencies(Main
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  controller_manager_msgs
  Eigen3
  orocos_kdl
  kdl_parser
)

# Install executable
install(TARGETS
  Main
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
