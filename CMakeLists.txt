cmake_minimum_required(VERSION 3.8)
project(scout_mini_nav)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# FAST-LIO IMU 位姿到车体位姿的转换节点，不连接底盘硬件。
add_executable(lidar_odom_transformer src/lidar_odom_transformer.cpp)
target_compile_features(lidar_odom_transformer PRIVATE cxx_std_17)
target_include_directories(lidar_odom_transformer PRIVATE include)
target_link_libraries(lidar_odom_transformer Eigen3::Eigen)
ament_target_dependencies(lidar_odom_transformer rclcpp nav_msgs geometry_msgs tf2_ros)
install(TARGETS lidar_odom_transformer DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# 安装启动文件、导航配置与车体里程计转换配置。
install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  # 用隔离 ROS 域的合成数据验证位姿与协方差，不启动真实底盘或雷达。
  add_test(NAME lidar_odom_transformer_geometry
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_lidar_odom_transformer.py
      --binary $<TARGET_FILE:lidar_odom_transformer>)
  set_tests_properties(lidar_odom_transformer_geometry PROPERTIES TIMEOUT 45)
  # 仅运行软件转换器和 EKF，验证两路测量选择、延迟融合与输出 TF。
  add_test(NAME wheel_lio_fusion
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_odometry_fusion.py
      --binary $<TARGET_FILE:lidar_odom_transformer>)
  set_tests_properties(wheel_lio_fusion PROPERTIES TIMEOUT 45)
endif()

ament_package()
