cmake_minimum_required(VERSION 3.8)
project(ros2_dual_stepper_controller LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

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

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
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
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
)


ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_encoder_stream test/test_encoder_stream.cpp)
  target_link_libraries(test_encoder_stream
          ros2_dual_stepper_controller
  )
endif()
