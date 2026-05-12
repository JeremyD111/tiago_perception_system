#include "pointcloud_concatenate/pointcloud_concatenate.hpp"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>

PointcloudConcatenate::PointcloudConcatenate()
: Node("pointcloud_concatenate")
{
  handleParams();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // QoS Estándar para evitar incompatibilidades de conexión
  rclcpp::QoS qos(10);

  sub_cloud_in1_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_in1", qos, std::bind(&PointcloudConcatenate::subCallbackCloudIn1, this, std::placeholders::_1));
  sub_cloud_in2_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_in2", qos, std::bind(&PointcloudConcatenate::subCallbackCloudIn2, this, std::placeholders::_1));
  sub_cloud_in3_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_in3", qos, std::bind(&PointcloudConcatenate::subCallbackCloudIn3, this, std::placeholders::_1));
  sub_cloud_in4_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_in4", qos, std::bind(&PointcloudConcatenate::subCallbackCloudIn4, this, std::placeholders::_1));

  pub_cloud_out_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_out", 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / hz_)),
    std::bind(&PointcloudConcatenate::update, this));
}

void PointcloudConcatenate::handleParams()
{
  declare_parameter<std::string>("target_frame", "base_link");
  get_parameter("target_frame", target_frame_);

  declare_parameter<int>("clouds", 2);
  get_parameter("clouds", clouds_);

  declare_parameter<double>("hz", 10.0);
  get_parameter("hz", hz_);

  RCLCPP_INFO(get_logger(), "Params — target_frame: %s, clouds: %d, hz: %.1f", target_frame_.c_str(), clouds_, hz_);
}

void PointcloudConcatenate::subCallbackCloudIn1(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  cloud_in1_ = *msg; cloud_in1_received_ = true;
}
void PointcloudConcatenate::subCallbackCloudIn2(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  cloud_in2_ = *msg; cloud_in2_received_ = true;
}
void PointcloudConcatenate::subCallbackCloudIn3(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  cloud_in3_ = *msg; cloud_in3_received_ = true;
}
void PointcloudConcatenate::subCallbackCloudIn4(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  cloud_in4_ = *msg; cloud_in4_received_ = true;
}

void PointcloudConcatenate::update()
{
  if (clouds_ < 1) return;

  // RADAR DE DIAGNÓSTICO: Si falta alguna nube, avisa a la consola cada ~2 segundos (20 ciclos a 10hz)
  if ((clouds_ >= 1 && !cloud_in1_received_) || (clouds_ >= 2 && !cloud_in2_received_)) {
      static int wait_count = 0;
      if (wait_count % 20 == 0) {
          RCLCPP_WARN(this->get_logger(), "Esperando datos... Nube 1 (Filtro): [%s] | Nube 2 (TimeBridge): [%s]", 
              cloud_in1_received_ ? "OK" : "FALTA", 
              cloud_in2_received_ ? "OK" : "FALTA");
      }
      wait_count++;
      return;
  }

  auto start_time = std::chrono::high_resolution_clock::now();

  sensor_msgs::msg::PointCloud2 local_c[4] = {cloud_in1_, cloud_in2_, cloud_in3_, cloud_in4_};
  
  size_t max_points = 0;
  for (int i = 0; i < clouds_; ++i) {
    max_points += (local_c[i].width * local_c[i].height);
  }

  pcl::PointCloud<pcl::PointXYZ> pcl_base;
  pcl_base.reserve(max_points);

  int nubes_fusionadas = 0;

  for (int i = 0; i < clouds_; ++i) {
    const auto& msg = local_c[i];
    if (msg.width * msg.height == 0) continue;

    try {
      geometry_msgs::msg::TransformStamped t_stamped = tf_buffer_->lookupTransform(
        target_frame_, msg.header.frame_id, tf2::TimePointZero);
      
      Eigen::Isometry3d iso = tf2::transformToEigen(t_stamped);
      Eigen::Matrix4f tf_matrix = iso.matrix().cast<float>();

      int x_off = -1, y_off = -1, z_off = -1;
      for (const auto& field : msg.fields) {
        if (field.name == "x") x_off = field.offset;
        else if (field.name == "y") y_off = field.offset;
        else if (field.name == "z") z_off = field.offset;
      }
      
      if (x_off < 0 || y_off < 0 || z_off < 0) continue;

      const uint8_t* point_ptr = msg.data.data();
      size_t point_step = msg.point_step;
      size_t num_points = msg.width * msg.height;

      for (size_t p = 0; p < num_points; ++p) {
        float x = *reinterpret_cast<const float*>(point_ptr + x_off);
        float y = *reinterpret_cast<const float*>(point_ptr + y_off);
        float z = *reinterpret_cast<const float*>(point_ptr + z_off);

        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
          Eigen::Vector4f pt(x, y, z, 1.0f);
          Eigen::Vector4f pt_trans = tf_matrix * pt;
          pcl_base.emplace_back(pt_trans.x(), pt_trans.y(), pt_trans.z());
        }
        point_ptr += point_step;
      }
      nubes_fusionadas++;

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Error TF en nube %d: %s", i+1, ex.what());
    }
  }

  if (!pcl_base.empty()) {
    sensor_msgs::msg::PointCloud2 final_output;
    pcl::toROSMsg(pcl_base, final_output);
    final_output.header.stamp = this->get_clock()->now();
    final_output.header.frame_id = target_frame_;
    pub_cloud_out_->publish(final_output);
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0;
  
  RCLCPP_INFO(this->get_logger(), "[RENDIMIENTO] Nubes fusionadas: %d/%d | Puntos: %zu | Tiempo: %.2f ms", 
              nubes_fusionadas, clouds_, pcl_base.size(), duration);
}

void PointcloudConcatenate::publishPointcloud(sensor_msgs::msg::PointCloud2 &cloud) {}