cmake_minimum_required(VERSION 3.8)
project(ros2_dual_stepper_controller)

include(ros2_control_cmake)
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(controller_manager 4.0.0 REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
)

foreach(dep ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
        src/dual_stepper_hardware_interface.cpp
        src/wheel_joint.cpp
        src/serial_comm.cpp
)

target_compres($ile_featu{PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface ros2_dual_stepper_controller.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(FILES ros2_dual_stepper_controller.xml DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description
)


ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
