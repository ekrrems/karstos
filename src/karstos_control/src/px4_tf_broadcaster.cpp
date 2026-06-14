#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class Px4TfBroadcaster : public rclcpp::Node {
public:
    Px4TfBroadcaster() : Node("px4_tf_broadcaster") {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        rclcpp::QoS qos_profile(10);
        qos_profile.best_effort();

        sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            qos_profile,
            std::bind(&Px4TfBroadcaster::callback, this, std::placeholders::_1));

        laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&Px4TfBroadcaster::cb, this, std::placeholders::_1));

        pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lidar_points", 10);
    }

private:
    void callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {

        geometry_msgs::msg::TransformStamped tf;

        tf.header.stamp = this->now();
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";

        tf.transform.translation.x = msg->position[0];
        tf.transform.translation.y = msg->position[1];
        tf.transform.translation.z = - msg->position[2];

        tf.transform.rotation.x = msg->q[1];
        tf.transform.rotation.y = msg->q[2];
        tf.transform.rotation.z = msg->q[3];
        tf.transform.rotation.w = msg->q[0];

        tf_broadcaster_->sendTransform(tf);
    }

    void cb(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = scan->header;
        cloud.header.frame_id = "lidar_link";
        cloud.height = 1;
        cloud.width = scan->ranges.size();
        cloud.is_dense = false;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");

        for (size_t i = 0; i < scan->ranges.size(); i++) {
            float r = scan->ranges[i];
            float angle = scan->angle_min + i * scan->angle_increment;
            if (r < scan->range_min || r > scan->range_max) {
                *x = *y = *z = std::numeric_limits<float>::quiet_NaN();
            } else {
                *x = r * cos(angle);
                *y = r * sin(angle);
                *z = 0.0;
            }
            ++x; ++y; ++z;
        }
        pc_pub_->publish(cloud);
    }

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4TfBroadcaster>());
    rclcpp::shutdown();

    return 0;
}