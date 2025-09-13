cmake_minimum_required(VERSION 3.8)
project(space_teams_definitions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Float.srv"
  "srv/String.srv"
  DEPENDENCIES # Add any dependencies here if needed
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
