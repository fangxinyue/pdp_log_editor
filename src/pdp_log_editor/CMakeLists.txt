cmake_minimum_required(VERSION 3.0.2)
project(pdp_log_editor)

# 1. 查找所有必需的包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  std_msgs
  message_generation
  pluginlib
  rosbag
)
find_package(nlohmann_json REQUIRED)
find_package(Qt5 REQUIRED Widgets)

# 2. 添加消息文件
add_message_files(
  FILES
  PdpLogAnnotation.msg
)

# 3. 生成消息
generate_messages(
  DEPENDENCIES
  std_msgs
)

# 4. 配置 catkin 包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rviz message_runtime std_msgs pluginlib
)

# 5. 设置 Qt MOC, UIC, RCC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# 6. 添加包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 7. 构建库
add_library(${PROJECT_NAME}
  src/pdp_log_editor_panel.cpp
  src/pdp_log_editor_tool.cpp
  include/pdp_log_editor/pdp_log_editor_panel.h
  include/pdp_log_editor/pdp_log_editor_tool.h
)

# 8. 添加对消息生成的依赖
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

# 9. 链接库
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Widgets
  nlohmann_json::nlohmann_json
)

# 10. 安装规则
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)