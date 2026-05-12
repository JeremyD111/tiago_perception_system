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
#include <thread>

// Msg and srv 
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"

// vamp
#include <vamp/collision/filter.hh>

using namespace std::chrono_literals;
using std::placeholders::_1;

class PointcloudClass : public rclcpp::Node
{
public:
    PointcloudClass()
    : Node("pointcloud_node")
    {
        // Arguments
        this->declare_parameter<std::string>("sub_pointcloud_topic", "/oakd/depth_camera/points");
        this->get_parameter("sub_pointcloud_topic", sub_pointcloud_topic);
        
        this->declare_parameter<bool>("simulation", true);
        this->get_parameter("simulation", simulation);

        // Filtering arguments
        this->declare_parameter<float>("filter_radius", 0.02);
        this->get_parameter("filter_radius", filter_radius);

        this->declare_parameter<float>("filter_cull_radius", 1.2 + 0.35); // Max extension radius for UR5
        this->get_parameter("filter_cull_radius", filter_cull_radius);

        this->declare_parameter<std::vector<double>>("filter_origin_vec", {0.0, 0.0, 0.0}); 
        this->get_parameter("filter_origin_vec", filter_origin_vec);

        this->declare_parameter<bool>("filter_cull", true);
        this->get_parameter("filter_cull", filter_cull);

        for (int i=0 ; i<3 ; i++)
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

        // Subscribers
        subscriber_pc_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          sub_pointcloud_topic, 5,
          std::bind(&PointcloudClass::subscriber_pc_callback, this, _1));

        // TF listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void subscriber_pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    { 
        start = this->get_clock()->now();
        if (first_time) {
            start_previous = start;
            first_time = !first_time;
        }
        
        original_pointcloud.clear();
        transformed_pointcloud.clear();

        // /////////////////////////////////////////////////
        // unpack pointcloud
        // /////////////////////////////////////////////////
        rclcpp::Time start_unpack = this->get_clock()->now();
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z){
            dummy_point[0] = *iter_x;
            dummy_point[1] = *iter_y;
            dummy_point[2] = *iter_z;

            has_nan = false;
            for (float val : dummy_point) {
                if (std::isnan(val)) {
                    has_nan = true;
                    break;
                }
            }
            if (has_nan){ continue; }
            original_pointcloud.push_back(dummy_point);
        }
        rclcpp::Time end_unpack = this->get_clock()->now();
        auto time_unpack = end_unpack - start_unpack;
        RCLCPP_INFO(this->get_logger(), "Unpack time: %ld ms", time_unpack.nanoseconds()/1000000);

        // /////////////////////////////////////////////////
        // trasnform to the torso lift link
        // /////////////////////////////////////////////////
        start_TF = this->get_clock()->now();
        try {
            tf_eavesdropper = tf_buffer_->lookupTransform(
                to_frame, from_frame, 
                tf2::TimePointZero);
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
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s", 
                to_frame.c_str(), from_frame.c_str(), ex.what());
            return;
        }

        for (const auto &point : original_pointcloud)
        {
            non_finite = false;
            for (std::size_t k = 0; k < 3; ++k) {
                if (!std::isfinite(point[k])) {
                    non_finite = true;
                }
            }
            if (non_finite) { continue; }

            point_left << point[0], point[1], point[2];
            transformed_point_left = T_head_torso * point_left;
            transformed_pointcloud.push_back({transformed_point_left.x(), 
                transformed_point_left.y(), transformed_point_left.z()});
        }
        end_TF = this->get_clock()->now();
        auto time_calculateddd = end_TF - start_TF;
        RCLCPP_INFO(this->get_logger(), "TF time: %ld ms", time_calculateddd.nanoseconds()/1000000);

        // /////////////////////////////////////////////////
        // Filter original pointcloud to volume around robot
        // /////////////////////////////////////////////////
        rclcpp::Time start_filtering_volume = this->get_clock()->now();
        
        filtered_pointcloud = vamp::collision::filter_pointcloud(
            transformed_pointcloud,
            filter_radius, 
            filter_cull_radius, 
            filter_origin, 
            bbox_lo, 
            bbox_hi, 
            filter_cull
        );
        
        rclcpp::Time end_filtering_volume = this->get_clock()->now();
        auto total_filtering_volume = end_filtering_volume - start_filtering_volume;
        RCLCPP_INFO(this->get_logger(), "Filtering volume time: %ld ms", total_filtering_volume.nanoseconds()/1000000);

        // /////////////////////////////////////////////////
        // Publish pointclouds 
        // /////////////////////////////////////////////////
        rclcpp::Time start_msg = this->get_clock()->now();
        
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
        
        rclcpp::Time end_msg = this->get_clock()->now();
        auto msg_time = end_msg - start_msg;
        RCLCPP_INFO(this->get_logger(), "Message time: %ld ms", msg_time.nanoseconds()/1000000);

        rclcpp::Time start_send = this->get_clock()->now();
        publisher_filtered_pc_->publish(output_msg);
        rclcpp::Time end_send = this->get_clock()->now();
        auto send_time = end_send - start_send;
        RCLCPP_INFO(this->get_logger(), "Publish time: %ld ms", send_time.nanoseconds()/1000000);
        RCLCPP_INFO(this->get_logger(), "-------------------------------");

        end = this->get_clock()->now();
        auto message_time = start - msg->header.stamp;
        auto process_time = end - start;
        auto callback_total_time = message_time + process_time;
        auto msg_frequency_time = start - start_previous;
        start_previous = start;

        RCLCPP_INFO(this->get_logger(), "Pointcloud message time: %ld ms, "
                                        "processing time: %ld ms, total time: %ld ms, "
                                        "time between pc: %ld, "
                                        "original pc: %li, "
                                        "filtered pc: %li",
            message_time.nanoseconds()/1000000, 
            process_time.nanoseconds()/1000000, 
            callback_total_time.nanoseconds()/1000000, 
            msg_frequency_time.nanoseconds()/1000000,
            original_pointcloud.size(),
            filtered_pointcloud.size()
        );
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_filtered_pc_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_pc_;
    
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
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointcloudClass>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}