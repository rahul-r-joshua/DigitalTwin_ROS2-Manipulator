cmake_minimum_required(VERSION 3.8)
project(arduinobot_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

ament_python_install_package(${PROJECT_NAME})

install (
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/live_hardware_controller.py
  ${PROJECT_NAME}/gui_hardware_controller.py
  ${PROJECT_NAME}/joystick_hardware_controller.py

  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/live_hardware_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME live_hardware_controller
)

install(PROGRAMS
  ${PROJECT_NAME}/gui_hardware_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gui_hardware_controller
)

install(PROGRAMS
  ${PROJECT_NAME}/joystick_hardware_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joystick_hardware_controller
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
