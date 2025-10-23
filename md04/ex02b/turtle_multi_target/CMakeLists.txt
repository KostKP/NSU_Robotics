cmake_minimum_required(VERSION 3.8)
project(turtle_multi_target)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# === Генерация сообщений ===
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CurrentTarget.msg"
)

# === Исполняемые файлы ===
add_executable(turtle_tf2_broadcaster src/turtle_tf2_broadcaster.cpp)
ament_target_dependencies(turtle_tf2_broadcaster rclcpp geometry_msgs tf2_ros tf2_geometry_msgs turtlesim)

add_executable(target_switcher src/target_switcher.cpp)
ament_target_dependencies(target_switcher rclcpp tf2_ros geometry_msgs turtlesim)

add_executable(turtle_controller src/turtle_controller.cpp)
ament_target_dependencies(turtle_controller rclcpp tf2_ros geometry_msgs turtlesim std_msgs)

# === Линковка с типами сообщений ===
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(target_switcher "${cpp_typesupport_target}")
target_link_libraries(turtle_controller "${cpp_typesupport_target}")

# === Установка ===
install(TARGETS
  turtle_tf2_broadcaster
  target_switcher
  turtle_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

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
