cmake_minimum_required(VERSION 3.8)
project(mtrackercar)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)



if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/CarRawCommand.msg"
    "msg/ControllerBufferSample.msg"
    "msg/DriveSample.msg"
    "srv/SetRegulator.srv"
    "srv/ControllerCapture.srv"
)
rosidl_get_typesupport_target(car_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
find_package(diagnostic_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(mtrackercar_core
    src/communication/protocol.cpp
    src/communication/serial_port.cpp
)

target_include_directories(mtrackercar_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

add_executable(mtrackercar_drv src/nodes/mtrackercar.cpp)
target_link_libraries(mtrackercar_drv mtrackercar_core "${car_typesupport}")
ament_target_dependencies(mtrackercar_drv
    ackermann_msgs
    diagnostic_msgs
    geometry_msgs
    nav_msgs
    rclcpp
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros
)

add_executable(controller_simple src/control/controller_simple.cpp)
ament_target_dependencies(controller_simple
    geometry_msgs
    rclcpp
)

add_executable(mtrackercar_joy src/nodes/mtrackercar_joy.cpp)
ament_target_dependencies(mtrackercar_joy geometry_msgs rclcpp sensor_msgs)

add_executable(mtrackercar_sim src/nodes/mtrackercar_sim.cpp)
target_include_directories(mtrackercar_sim PRIVATE include)
ament_target_dependencies(mtrackercar_sim
    diagnostic_msgs geometry_msgs nav_msgs rclcpp tf2 tf2_geometry_msgs tf2_ros)

add_executable(trajectory_demo src/nodes/trajectory_demo.cpp)
target_include_directories(trajectory_demo PRIVATE include)
ament_target_dependencies(trajectory_demo geometry_msgs nav_msgs rclcpp)

add_executable(controller_tracking src/control/controller_tracking.cpp)
target_include_directories(controller_tracking PRIVATE include)
ament_target_dependencies(controller_tracking geometry_msgs nav_msgs rclcpp tf2 tf2_ros)

add_executable(set_odometry_demo src/nodes/set_odometry_demo.cpp)
ament_target_dependencies(set_odometry_demo
    geometry_msgs
    rclcpp
)

install(
    TARGETS mtrackercar_core mtrackercar_drv controller_simple set_odometry_demo
        trajectory_demo controller_tracking mtrackercar_joy mtrackercar_sim
    EXPORT export_mtrackercar
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(
    DIRECTORY config launch doc
    DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/steering_calibration.py
    DESTINATION lib/${PROJECT_NAME} RENAME steering_calibration)
install(PROGRAMS scripts/compact_demo.py
    DESTINATION lib/${PROJECT_NAME} RENAME compact_demo)
install(FILES scripts/calibration_identification.py scripts/regulator_experiment.py
    scripts/trace_data.py
    DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/regulator_tuning.py
    DESTINATION lib/${PROJECT_NAME} RENAME regulator_tuning)
install(PROGRAMS scripts/power_monitor.py
    DESTINATION lib/${PROJECT_NAME} RENAME pc_power_monitor)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_cmake_cppcheck REQUIRED)
    find_package(ament_cmake_cpplint REQUIRED)
    find_package(ament_cmake_flake8 REQUIRED)
    find_package(ament_cmake_lint_cmake REQUIRED)
    find_package(ament_cmake_pep257 REQUIRED)
    find_package(ament_cmake_uncrustify REQUIRED)
    find_package(ament_cmake_xmllint REQUIRED)

    ament_cppcheck()
    ament_cpplint(FILTERS "-legal/copyright")
    ament_flake8()
    ament_lint_cmake()
    ament_pep257()
    ament_uncrustify()
    ament_xmllint()

    add_executable(test_protocol test/test_protocol.cpp)
    add_test(NAME test_protocol COMMAND test_protocol)
    target_link_libraries(test_protocol mtrackercar_core)
    add_executable(test_car_model test/test_car_model.cpp)
    target_include_directories(test_car_model PRIVATE include)
    add_test(NAME test_car_model COMMAND test_car_model)
    ament_add_pytest_test(test_regulator_experiment test/test_regulator_experiment.py)
    ament_add_pytest_test(test_power_monitor test/test_power_monitor.py)
    ament_add_pytest_test(test_compact_demo test/test_compact_demo.py)
    ament_add_pytest_test(test_calibration test/test_calibration.py)
    ament_add_pytest_test(test_circle_demo test/test_circle_demo.py)
    ament_add_gtest(test_demo_control test/test_demo_control.cpp)
    target_include_directories(test_demo_control PRIVATE include)
    ament_target_dependencies(test_demo_control geometry_msgs nav_msgs)
endif()

ament_export_targets(export_mtrackercar HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
