// Pose extractor
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class PoseExtractor : public rclcpp::Node {
    public:
        PoseExtractor() : Node("pose_extractor") {
            sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
                "/gazebo_poses", 10,
            [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
                for (auto& t : msg->transforms) {
                    if (t.child_frame_id == "my_drone") {
                        geometry_msgs::msg::PoseWithCovarianceStamped pose;
                        pose.header = t.header;
                        pose.header.frame_id = "map";
                        pose.pose.pose.position.x = t.transform.translation.x;
                        pose.pose.pose.position.y = t.transform.translation.y;
                        pose.pose.pose.position.z = t.transform.translation.z;
                        pose.pose.pose.orientation = t.transform.rotation;
                        pub_->publish(pose);
                    }
                }
            });

            pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose", 10);
        }
    
    private:
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseExtractor>());
    rclcpp::shutdown();
    return 0;
}