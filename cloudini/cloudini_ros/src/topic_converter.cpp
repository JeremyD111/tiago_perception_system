/*
 * Copyright 2025 Davide Faconti
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cloudini_lib/cloudini.hpp>
#include <cloudini_lib/ros_msg_utils.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <fstream>
#include <vector>
#include <iomanip> // Para la precisión de los decimales
#include <chrono>
#include <iostream> // Para std::cout y std::cerr

/*
 * This node converts compressed point cloud messages from the
 * `point_cloud_interfaces/msg/CompressedPointCloud2` format to the
 * `sensor_msgs/msg/PointCloud2` format.
 *
 * It is BRUTALLY efficient, because we reade directly the RAW DDS message,
 * and write the output message without any intermediate copies.
 *
 * This means less CPU and less latency.
 */
class CloudiniPointcloudConverter : public rclcpp::Node {
 public:
  CloudiniPointcloudConverter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  void callback(std::shared_ptr<rclcpp::SerializedMessage> msg);

  ~CloudiniPointcloudConverter() {
    std::cout << "\n[SISTEMA] Apagando nodo... Guardando " << historial_latencia_.size() << " registros...\n";

    // Archivo general para Tiago (se renombrará manualmente según la prueba)
    std::string ruta_archivo = "/home/users/jeremy.delgado/workspace_local/historial_tiago.csv";

    std::ofstream archivo_csv(ruta_archivo);
    if (archivo_csv.is_open()) {
        archivo_csv << "Timestamp,Latencia_ms,Descompresion_ms\n";

        for (const auto& registro : historial_latencia_) {
            archivo_csv << std::fixed << std::setprecision(6)
                        << registro.timestamp << "," << registro.latencia << "," << registro.descompresion << "\n";
        }
        archivo_csv.close();
        std::cout << "[SISTEMA] CSV guardado exitosamente en: " << ruta_archivo << "\n";
    } else {
        std::cerr << "[ERROR] No se pudo crear el archivo CSV.\n";
    }

    // bypass the deleter
    output_message_.get_rcl_serialized_message().buffer = nullptr;
    output_message_.get_rcl_serialized_message().buffer_length = 0;
  }

 private:
  // generic subscriber for compressed point cloud messages
  rclcpp::GenericSubscription::SharedPtr point_cloud_subscriber_;

  // generic publisher for sensor_msgs/msg/PointCloud2 (but... raw DDS message)
  rclcpp::GenericPublisher::SharedPtr point_cloud_publisher_;

  // callback for point cloud messages
  void point_cloud_callback(const rclcpp::SerializedMessage& serialized_msg);

  std::vector<uint8_t> output_raw_message_;
  rclcpp::SerializedMessage output_message_;
  bool compressing_ = true;
  double resolution_ = 0.001;  // 1mm

  uint64_t tot_original_size = 0;
  uint64_t tot_compressed_size = 0;

  struct RegistroLatencia {
    double timestamp;
    double latencia;
    double descompresion;
  };
  std::vector<RegistroLatencia> historial_latencia_;
};
//-----------------------------------------------------

rclcpp::QoS adapt_request_to_offers(
    const std::string& topic_name, const std::vector<rclcpp::TopicEndpointInfo>& endpoints) {
  rclcpp::QoS request_qos(rmw_qos_profile_default.depth);

  if (endpoints.empty()) {
    return request_qos;
  }
  size_t reliability_reliable_endpoints_count = 0;
  size_t durability_transient_local_endpoints_count = 0;
  for (const auto& endpoint : endpoints) {
    const auto& profile = endpoint.qos_profile().get_rmw_qos_profile();
    if (profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
      reliability_reliable_endpoints_count++;
    }
    if (profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
      durability_transient_local_endpoints_count++;
    }
  }
  // Policy: reliability
  if (reliability_reliable_endpoints_count == endpoints.size()) {
    request_qos.reliable();
  } else {
    request_qos.best_effort();
  }
  // Policy: durability
  // If all publishers offer transient_local, we can request it and receive latched messages
  if (durability_transient_local_endpoints_count == endpoints.size()) {
    request_qos.transient_local();
  } else {
    request_qos.durability_volatile();
  }
  return request_qos;
}
//-----------------------------------------------------

CloudiniPointcloudConverter::CloudiniPointcloudConverter(const rclcpp::NodeOptions& options)
    : rclcpp::Node("cloudini_pointcloud_converter", rclcpp::NodeOptions(options).use_intra_process_comms(true)) {

  // Pre-asignar memoria para evitar latencia por realocación dinámica
  historial_latencia_.reserve(10000);

  // Declare parameters for input and output topics
  this->declare_parameter<bool>("compressing", true);
  this->declare_parameter<std::string>("topic_input", "/points");
  this->declare_parameter<std::string>("topic_output", "");
  this->declare_parameter<double>("resolution", 0.001);

  // read parameters
  compressing_ = this->get_parameter("compressing").as_bool();
  resolution_ = this->get_parameter("resolution").as_double();

  const std::string input_topic = this->get_parameter("topic_input").as_string();
  if (input_topic.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Input topic is not set");
    throw std::runtime_error("Input topic is not set");
  }
  std::string output_topic = this->get_parameter("topic_output").as_string();
  if (output_topic.empty()) {
    output_topic = input_topic + (compressing_ ? "/compressed" : "/decompressed");
    RCLCPP_WARN(this->get_logger(), "Output topic is not set, using default: %s", output_topic.c_str());
  }

  // Initialize point cloud type support
  // Forzar QoS Best Effort (Cola de 1) para emparejar estrictamente con Sputnik y medir tiempo real puro
  rclcpp::QoS sub_qos(1);
  sub_qos.best_effort();
  sub_qos.durability_volatile();

  // QoS para publicar la nube reconstruida dentro del Tiago
  rclcpp::QoS pub_qos(10);

  std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback =
      std::bind(&CloudiniPointcloudConverter::callback, this, std::placeholders::_1);

  const std::string compressed_topic_type = "point_cloud_interfaces/msg/CompressedPointCloud2";
  const std::string pointcloud_topic_type = "sensor_msgs/msg/PointCloud2";

  const std::string input_topic_type = compressing_ ? pointcloud_topic_type : compressed_topic_type;
  const std::string output_topic_type = compressing_ ? compressed_topic_type : pointcloud_topic_type;

  RCLCPP_INFO(
      this->get_logger(), "Subscribing to topic '%s' of type '%s'", input_topic.c_str(), input_topic_type.c_str());

  // Create a generic subscriber for point cloud messages
  point_cloud_subscriber_ = this->create_generic_subscription(
      input_topic,       //
      input_topic_type,  //
      sub_qos,           //
      callback);

  RCLCPP_INFO(
      this->get_logger(), "Publishing to topic '%s' of type '%s'", output_topic.c_str(), output_topic_type.c_str());
  // Create a generic publisher for point cloud messages
  point_cloud_publisher_ = this->create_generic_publisher(output_topic, output_topic_type, pub_qos);
}

void CloudiniPointcloudConverter::callback(std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  // Skip processing if there are no subscribers
  // ANULADO PARA FORZAR LA MEDICIÓN DE LATENCIA
  //if (point_cloud_publisher_->get_subscription_count() == 0) {
  //  return;
  //}

  // STEP 1: convert the buffer to a ConstBufferView (this is not a copy)
  const auto& input_msg = msg->get_rcl_serialized_message();
  const Cloudini::ConstBufferView raw_dds_msg(input_msg.buffer, input_msg.buffer_length);

  // STEP 2: extract information from the raw DDS message
  auto pc_info = cloudini_ros::getDeserializedPointCloudMessage(raw_dds_msg);

  // --- INICIO DE MEDICIÓN ---
  auto start_time = std::chrono::high_resolution_clock::now();

  double stamp_envio_exacto = 0.0;
  double latencia_cruda_ms = 0.0;

  if (compressing_) {
    // ==============================================================
    // LADO SPUTNIK (COMPRESIÓN)
    // ==============================================================
    auto time_now = this->get_clock()->now();
    pc_info.ros_header.stamp_sec = time_now.seconds();
    pc_info.ros_header.stamp_nsec = time_now.nanoseconds() % 1000000000;

    cloudini_ros::applyResolutionProfile(cloudini_ros::ResolutionProfile{}, pc_info.fields, resolution_);
    const auto encoding_info = cloudini_ros::toEncodingInfo(pc_info);
    cloudini_ros::convertPointCloud2ToCompressedCloud(pc_info, encoding_info, output_raw_message_);
  } else {
    // ==============================================================
    // LADO TIAGO (DESCOMPRESIÓN)
    // ==============================================================
    auto tiempo_llegada = this->get_clock()->now();
    rclcpp::Time tiempo_envio(pc_info.ros_header.stamp_sec, pc_info.ros_header.stamp_nsec, RCL_ROS_TIME);

    latencia_cruda_ms = (tiempo_llegada - tiempo_envio).seconds() * 1000.0;

    // Reconstruir el timestamp exacto como llave primaria
    stamp_envio_exacto = pc_info.ros_header.stamp_sec + (pc_info.ros_header.stamp_nsec / 1e9);

    cloudini_ros::convertCompressedCloudToPointCloud2(pc_info, output_raw_message_);

    static int lat_count = 0;
    if (lat_count % 20 == 0) {
        RCLCPP_INFO(this->get_logger(), "[LATENCIA DE RED] Viaje Sputnik -> Tiago: %.2f ms", latencia_cruda_ms);
    }
    lat_count++;
  }

  // --- FIN DE MEDICIÓN ---
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

  // Volcar latencia y descompresión a la RAM
  if (!compressing_) {
      double descompresion_ms = static_cast<double>(duration) / 1000.0;
      historial_latencia_.push_back({stamp_envio_exacto, latencia_cruda_ms, descompresion_ms});
  }

  // STEP 3: publish the output message (Con escudo para el apagado)
  output_message_.get_rcl_serialized_message().buffer_length = output_raw_message_.size();
  output_message_.get_rcl_serialized_message().buffer = output_raw_message_.data();

  if (rclcpp::ok()) {
      point_cloud_publisher_->publish(output_message_);
  } else {
      std::cout << "[SISTEMA] Ignorando publicación, ROS 2 se está apagando...\n";
  }

  tot_original_size += input_msg.buffer_length;
  tot_compressed_size += output_raw_message_.size();

  static int count = 0;
  if (count % 20 == 0)
  {
    double average_ratio = static_cast<double>(tot_compressed_size) / tot_original_size;
    tot_compressed_size = 0;
    tot_original_size = 0;

    RCLCPP_INFO(this->get_logger(),
    "[%s] %d msgs | Ratio: %.2f | Tiempo de descompresion de un frame: %.3f ms",
    compressing_ ? "COMP" : "DECOMP",
    count, average_ratio, static_cast<double>(duration) / 1000.0);
  }
  count++;
}

RCLCPP_COMPONENTS_REGISTER_NODE(CloudiniPointcloudConverter)
