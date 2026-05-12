cmake_minimum_required(VERSION 3.16)
project(ros2_tiago_perception)

# ---------------------------------------------------------
# BANDERAS DE COMPILACIÓN (Máximo rendimiento para nubes de puntos)
# ---------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -mavx -mfma -O3 -march=native)
endif()

# ---------------------------------------------------------
# DEPENDENCIAS DEL SISTEMA
# ---------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cloudini_lib REQUIRED)
find_package(point_cloud_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)

# ---------------------------------------------------------
# DEPENDENCIAS LOCALES (VAMP)
# ---------------------------------------------------------
add_subdirectory(third_party/vamp)


# =========================================================
# NODO 1: Filtro TIAGo (Sin brazos)
# =========================================================
add_executable(vamp_filter_node2 src/vamp_filter_node2.cpp)
ament_target_dependencies(vamp_filter_node2 
  rclcpp sensor_msgs geometry_msgs tf2 tf2_ros Eigen3
)
# Se enlaza la librería compilada localmente
target_link_libraries(vamp_filter_node2 vamp::vamp)


# =========================================================
# NODO 2: Filtro y Compresión Sputnik (Cámara Orbbec)
# =========================================================
add_executable(vamp_cloudini_node src/vamp_cloudini_node.cpp)
ament_target_dependencies(vamp_cloudini_node 
  rclcpp sensor_msgs geometry_msgs tf2 tf2_ros Eigen3 cloudini_lib point_cloud_interfaces
)
# Este también usa VAMP
target_link_libraries(vamp_cloudini_node vamp::vamp)


# =========================================================
# NODO 3: Concatenador
# =========================================================
add_executable(pointcloud_concatenate_node 
  src/pointcloud_concatenate_node.cpp 
  src/pointcloud_concatenate.cpp
)
# Indicar dónde buscar el .hpp
target_include_directories(pointcloud_concatenate_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pointcloud_concatenate_node 
  rclcpp sensor_msgs tf2 tf2_ros tf2_eigen Eigen3 pcl_conversions
)


# =========================================================
# INSTALACIÓN
# =========================================================
install(TARGETS
  vamp_filter_node2
  vamp_cloudini_node
  pointcloud_concatenate_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()