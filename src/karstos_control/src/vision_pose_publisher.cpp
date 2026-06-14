#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <random>

class VisionPosePublisher : public rclcpp::Node {
public:
    VisionPosePublisher() : Node("vision_pose_publisher"),
        gen_(std::random_device{}()),
        noise_(0.0, 0.02)
    {
        // Subscribe to Gazebo dynamic poses
        sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
            "/gazebo_poses", 10,
            [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
                // x500_0 is always index 0 in dynamic_pose/info
                if (msg->poses.empty()) return;
                publish_to_px4(msg->poses[0]);
            });

        pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", 10);

        RCLCPP_INFO(get_logger(), "Vision pose publisher ready");
    }

private:
    void publish_to_px4(const geometry_msgs::msg::Pose& pose) {
        px4_msgs::msg::VehicleOdometry odom;
        odom.timestamp = now().nanoseconds() / 1000;
        odom.timestamp_sample = odom.timestamp;
        // odom.estimator_type = px4_msgs::msg::VehicleOdometry::ESTIMATOR_TYPE_VISION;
        odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

        // ENU → NED: North=y, East=x, Down=-z
        odom.position[0] =  (float)(pose.position.y + noise_(gen_));
        odom.position[1] =  (float)(pose.position.x + noise_(gen_));
        odom.position[2] = -(float)(pose.position.z + noise_(gen_));

        // Quaternion ENU→NED
        odom.q[0] =  (float)pose.orientation.w;
        odom.q[1] =  (float)pose.orientation.y;
        odom.q[2] =  (float)pose.orientation.x;
        odom.q[3] = -(float)pose.orientation.z;

        odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
        odom.velocity[0] = NAN;
        odom.velocity[1] = NAN;
        odom.velocity[2] = NAN;
        odom.angular_velocity[0] = NAN;
        odom.angular_velocity[1] = NAN;
        odom.angular_velocity[2] = NAN;
        odom.position_variance[0] = 0.0004f;
        odom.position_variance[1] = 0.0004f;
        odom.position_variance[2] = 0.0004f;
        odom.orientation_variance[0] = 0.0001f;
        odom.orientation_variance[1] = 0.0001f;
        odom.orientation_variance[2] = 0.0001f;
        odom.velocity_variance[0] = NAN;
        odom.velocity_variance[1] = NAN;
        odom.velocity_variance[2] = NAN;
        odom.reset_counter = 0;
        odom.quality = 100;

        pub_->publish(odom);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_;
    std::mt19937 gen_;
    std::normal_distribution<double> noise_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionPosePublisher>());
    rclcpp::shutdown();
}