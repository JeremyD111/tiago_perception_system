// ROS
#include "rclcpp/rclcpp.hpp"
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// cpp utils
#include <chrono>
#include <utility>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <cstdlib>
#include <Eigen/Dense>
#include <fstream> // Añadido para escritura de CSV

// Multi-threading
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

// Msg and srv 
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"

// Cloudini
#include <cloudini_lib/cloudini.hpp>
#include <cloudini_lib/ros_msg_utils.hpp>
#include <point_cloud_interfaces/msg/compressed_point_cloud2.hpp>

#include <rclcpp/serialization.hpp>

// vamp
#include <vamp/collision/filter.hh>

using namespace std::chrono_literals;
using std::placeholders::_1;

class PointcloudClass : public rclcpp::Node
{
public:
    PointcloudClass()
    : Node("sputnik_filter_node") // Corrección aplicada aquí
    {
        // Pre-asignar memoria para el registro de compresión
        historial_compresion_.reserve(10000);

        // Arguments
        this->declare_parameter<std::string>("sub_pointcloud_topic", "/oakd/depth_camera/points");
        this->get_parameter("sub_pointcloud_topic", sub_pointcloud_topic);
        
        this->declare_parameter<bool>("simulation", true);
        this->get_parameter("simulation", simulation);

        // Filtering arguments
        this->declare_parameter<float>("filter_radius", 0.02);
        this->get_parameter("filter_radius", filter_radius);

        this->declare_parameter<float>("filter_cull_radius", 1.2 + 0.35); 
        this->get_parameter("filter_cull_radius", filter_cull_radius);

        this->declare_parameter<std::vector<double>>("filter_origin_vec", {0.0, 0.0, 0.0}); 
        this->get_parameter("filter_origin_vec", filter_origin_vec);

        this->declare_parameter<bool>("filter_cull", true);
        this->get_parameter("filter_cull", filter_cull);

        int i;
        for (i=0 ; i<3 ; i++)
        {
            bbox_lo[i] = filter_origin_vec[i] - filter_cull_radius;
            bbox_hi[i] = filter_origin_vec[i] + filter_cull_radius;
            filter_origin[i] = filter_origin_vec[i];
        }

        // Transformation matrix
        mat_head_torso <<
                -0.000, -1.000,  0.000,  0.022,
                -0.000,  0.000, -1.000,  0.178,
                 1.000, -0.000, -0.000, -0.278,
                 0.000,  0.000,  0.000,  1.000;

        T_head_torso.matrix() = mat_head_torso;
        offset << 0.256, -0.156, 0.456;
        R_x = Eigen::AngleAxisf(M_PI/2.0f, Eigen::Vector3f::UnitX()).toRotationMatrix();
        R_z = Eigen::AngleAxisf(M_PI/2.0f, Eigen::Vector3f::UnitZ()).toRotationMatrix();

        T_head_torso.linear() = T_head_torso.linear() * R_x * R_z;
        T_head_torso.translation() += offset;

        // Publishers
        publisher_filtered_pc_ = create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud_topic", 10); 
        rclcpp::QoS qos_profile(1);
        qos_profile.best_effort();
        qos_profile.durability_volatile();

        publisher_compressed_pc_ = create_publisher<point_cloud_interfaces::msg::CompressedPointCloud2>(
            "/point_cloud_topic/compressed", qos_profile);
                
        this->declare_parameter<bool>("use_vamp_filter", true);
        this->get_parameter("use_vamp_filter", use_vamp_filter);
        
        this->declare_parameter<double>("compression_resolution", 0.001); 
        this->get_parameter("compression_resolution", compression_resolution);

        // Subscribers
        subscriber_pc_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          sub_pointcloud_topic, 5,
          std::bind(&PointcloudClass::subscriber_pc_callback, this, _1));

        // TF listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Iniciar hilo de compresión
        compression_thread_ = std::thread(&PointcloudClass::compression_worker, this);
    }

    ~PointcloudClass()
    {
        // Guardar CSV de compresión antes de apagar los hilos
        std::cout << "\n[SISTEMA] Guardando " << historial_compresion_.size() << " registros de compresion...\n";
        std::string ruta_archivo = "/home/users/jeremy.delgado/prueba_camara/historial_sputnik.csv";
        
        std::ofstream archivo_csv(ruta_archivo);
        if (archivo_csv.is_open()) {
            archivo_csv << "Timestamp,Compresion_ms\n";
            for (const auto& reg : historial_compresion_) {
                archivo_csv << std::fixed << std::setprecision(6) 
                            << reg.timestamp_envio << "," << reg.tiempo_compresion_ms << "\n";
            }
            archivo_csv.close();
            std::cout << "[SISTEMA] CSV guardado en: " << ruta_archivo << "\n";
        } else {
            std::cerr << "[ERROR] No se pudo crear el archivo CSV en Sputnik.\n";
        }

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            keep_running_ = false;
        }
        queue_cv_.notify_all();
        if (compression_thread_.joinable()) {
            compression_thread_.join();
        }
    }

    void subscriber_pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    { 
        auto start_total = this->get_clock()->now();
        sensor_msgs::msg::PointCloud2 cloud_to_compress;

        if (use_vamp_filter) {
            original_pointcloud.clear();
            transformed_pointcloud.clear();

            rclcpp::Time start_unpack = this->get_clock()->now();
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z){
                dummy_point[0] = *iter_x; dummy_point[1] = *iter_y; dummy_point[2] = *iter_z;
                has_nan = false;
                for (float val : dummy_point) {
                    if (std::isnan(val)) { has_nan = true; break; }
                }
                if (has_nan){ continue; }
                original_pointcloud.push_back(dummy_point);
            }
            auto time_unpack = this->get_clock()->now() - start_unpack;

            start_TF = this->get_clock()->now();
            try {
                tf_eavesdropper = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);
                T_head_torso.translation() <<
                    tf_eavesdropper.transform.translation.x,
                    tf_eavesdropper.transform.translation.y,
                    tf_eavesdropper.transform.translation.z;
                q.w() = tf_eavesdropper.transform.rotation.w;
                q.x() = tf_eavesdropper.transform.rotation.x;
                q.y() = tf_eavesdropper.transform.rotation.y;
                q.z() = tf_eavesdropper.transform.rotation.z;
                q.normalize();
                T_head_torso.linear() = q.toRotationMatrix();
            } catch (const tf2::TransformException & ex) {
                return;
            }

            for (const auto &point : original_pointcloud) {
                non_finite = false;
                std::size_t k;
                for (k = 0; k < 3; ++k) {
                    if (!std::isfinite(point[k])) { non_finite = true; break;}
                }
                if (non_finite) { continue; }

                point_left << point[0], point[1], point[2];
                transformed_point_left = T_head_torso * point_left;
                transformed_pointcloud.push_back({transformed_point_left.x(), transformed_point_left.y(), transformed_point_left.z()});
            }
            auto time_tf = this->get_clock()->now() - start_TF;

            rclcpp::Time start_filter = this->get_clock()->now();
            filtered_pointcloud = vamp::collision::filter_pointcloud(
                transformed_pointcloud, filter_radius, filter_cull_radius, filter_origin, bbox_lo, bbox_hi, filter_cull);
            auto time_filter = this->get_clock()->now() - start_filter;

            sensor_msgs::msg::PointCloud2 output_msg;
            output_msg.header = msg->header;
            output_msg.header.frame_id = to_frame;
            output_msg.height = 1;
            output_msg.width = filtered_pointcloud.size();
            output_msg.is_dense = true;
            output_msg.is_bigendian = false;

            sensor_msgs::PointCloud2Modifier mod(output_msg);
            mod.setPointCloud2FieldsByString(1, "xyz");
            mod.resize(filtered_pointcloud.size());

            sensor_msgs::PointCloud2Iterator<float> out_x(output_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> out_y(output_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> out_z(output_msg, "z");

            for (const auto& pt : filtered_pointcloud) {
                *out_x = pt[0]; *out_y = pt[1]; *out_z = pt[2];
                ++out_x; ++out_y; ++out_z;
            }

            publisher_filtered_pc_->publish(output_msg);
            cloud_to_compress = output_msg;

            RCLCPP_INFO(this->get_logger(), "[VAMP] Puntos Originales: %li | Filtrados: %li | Unpack: %ld ms | TF: %ld ms | Filter: %ld ms",
                        original_pointcloud.size(), filtered_pointcloud.size(), 
                        time_unpack.nanoseconds()/1000000, time_tf.nanoseconds()/1000000, time_filter.nanoseconds()/1000000);

        } else {
            cloud_to_compress = *msg;
        }

        // Empujar a la cola para el hilo de compresión
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (compression_queue_.size() < 3) {
                compression_queue_.push(cloud_to_compress);
            } else {
                RCLCPP_WARN(this->get_logger(), "Cola llena. Frame descartado para mantener tiempo real.");
            }
        }
        queue_cv_.notify_one();
    }

    void compression_worker()
    {
        while (keep_running_) {
            sensor_msgs::msg::PointCloud2 cloud_to_compress;
            
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait(lock, [this]{ return !compression_queue_.empty() || !keep_running_; });
                
                if (!keep_running_ && compression_queue_.empty()) break;
                
                cloud_to_compress = compression_queue_.front();
                compression_queue_.pop();
            }

            // 1. Preparación y Serialización
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
            rclcpp::SerializedMessage serialized_msg;
            serializer.serialize_message(&cloud_to_compress, &serialized_msg);

            const auto& rcl_msg = serialized_msg.get_rcl_serialized_message();
            const Cloudini::ConstBufferView raw_dds_msg(rcl_msg.buffer, rcl_msg.buffer_length);
            auto pc_info = cloudini_ros::getDeserializedPointCloudMessage(raw_dds_msg);

            std::vector<uint8_t> output_raw_message;
            cloudini_ros::applyResolutionProfile(cloudini_ros::ResolutionProfile{}, pc_info.fields, compression_resolution);
            const auto encoding_info = cloudini_ros::toEncodingInfo(pc_info);

            // 2. Sello de tiempo para el viaje por red
            auto time_pre_network = this->get_clock()->now();
            pc_info.ros_header.stamp_sec = time_pre_network.seconds();
            pc_info.ros_header.stamp_nsec = time_pre_network.nanoseconds() % 1000000000;

            // 3. INICIO DEL CRONÓMETRO DE COMPRESIÓN PURA
            auto start_compress = std::chrono::high_resolution_clock::now();

            cloudini_ros::convertPointCloud2ToCompressedCloud(pc_info, encoding_info, output_raw_message);

            // FIN DEL CRONÓMETRO DE COMPRESIÓN PURA
            auto end_compress = std::chrono::high_resolution_clock::now();

            rclcpp::SerializedMessage final_compressed_msg;
            final_compressed_msg.get_rcl_serialized_message().buffer_length = output_raw_message.size();
            final_compressed_msg.get_rcl_serialized_message().buffer = output_raw_message.data();

            if (rclcpp::ok()) {
                publisher_compressed_pc_->publish(final_compressed_msg);
            } else {
                std::cout << "[SISTEMA] Ignorando publicación final en Sputnik, ROS 2 se está apagando...\n";
            }

            final_compressed_msg.get_rcl_serialized_message().buffer = nullptr;
            final_compressed_msg.get_rcl_serialized_message().buffer_length = 0;

            auto duration_compress = std::chrono::duration_cast<std::chrono::microseconds>(end_compress - start_compress).count();

            // Guardar registro de compresión
            double stamp_envio_exacto = pc_info.ros_header.stamp_sec + (pc_info.ros_header.stamp_nsec / 1e9);
            double compresion_ms = static_cast<double>(duration_compress) / 1000.0;
            historial_compresion_.push_back({stamp_envio_exacto, compresion_ms});

            static int count = 0;
            if (count % 20 == 0) {
                double ratio = static_cast<double>(output_raw_message.size()) / rcl_msg.buffer_length;
                RCLCPP_INFO(this->get_logger(),
                    "[CLOUDINI] Origen: %s | Puntos a comprimir: %d | Ratio Red de Bytes: %.3f | T. Compresion: %.3f ms",
                    use_vamp_filter ? "Filtrado" : "Nube Original",
                    cloud_to_compress.width * cloud_to_compress.height,
                    ratio, static_cast<double>(duration_compress)/1000.0);
            }
            count++;
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_filtered_pc_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_pc_;
    
    rclcpp::Publisher<point_cloud_interfaces::msg::CompressedPointCloud2>::SharedPtr publisher_compressed_pc_;
    bool use_vamp_filter;
    double compression_resolution;

    std::string sub_pointcloud_topic;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string from_frame = "head_front_camera_color_optical_frame";
    std::string to_frame = "torso_lift_link";

    bool simulation;
    Eigen::Matrix4f mat_head_torso;
    Eigen::Affine3f T_head_torso;
    Eigen::Vector3f point_left;
    Eigen::Vector3f transformed_point_left;
    Eigen::Matrix3f R_x, R_z;
    Eigen::Quaternionf q;
    Eigen::Vector3f offset;
    bool non_finite = false;
    geometry_msgs::msg::TransformStamped tf_eavesdropper;

    std::array<float, 3> dummy_point;
    std::vector<std::array<float, 3>> original_pointcloud;
    std::vector<std::array<float, 3>> transformed_pointcloud;
    bool has_nan = false;

    std::vector<std::array<float, 3>> filtered_pointcloud;
    float filter_radius;
    float filter_cull_radius;
    std::array<float,3> filter_origin;
    std::vector<double> filter_origin_vec;
    std::array<float,3> bbox_lo;
    std::array<float,3> bbox_hi;
    bool filter_cull;

    rclcpp::Time start;
    rclcpp::Time end;
    rclcpp::Time start_previous;
    bool first_time = true;
    rclcpp::Time start_TF;
    rclcpp::Time end_TF;

    // Variables de Multi-threading
    std::queue<sensor_msgs::msg::PointCloud2> compression_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread compression_thread_;
    bool keep_running_ = true;

    struct RegistroCompresion {
        double timestamp_envio;
        double tiempo_compresion_ms;
    };
    std::vector<RegistroCompresion> historial_compresion_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointcloudClass>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}