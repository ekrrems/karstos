#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

class OffboardController : public rclcpp::Node {
public:
    OffboardController() : Node("offboard_controller") {
        offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        // Send setpoints at 10Hz — required by PX4 to stay in offboard mode
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { timer_cb(); });

        RCLCPP_INFO(get_logger(), "Offboard controller ready");
    }

private:
    void timer_cb() {
        // Must publish offboard control mode continuously
        px4_msgs::msg::OffboardControlMode mode;
        mode.timestamp = now().nanoseconds() / 1000;
        mode.position = true;
        mode.velocity = false;
        mode.acceleration = false;
        offboard_pub_->publish(mode);

        // Position setpoint — hover at 2m height
        px4_msgs::msg::TrajectorySetpoint sp;
        sp.timestamp = now().nanoseconds() / 1000;
        sp.position = {0.0f, 0.0f, -2.0f};  // NED: z negative = up
        sp.yaw = 0.0f;
        setpoint_pub_->publish(sp);

        counter_++;

        // After 50 cycles (5s) — arm and switch to offboard
        if (counter_ == 50) {
            arm();
            enable_offboard();
        }
    }

    void arm() {
        px4_msgs::msg::VehicleCommand cmd;
        cmd.timestamp = now().nanoseconds() / 1000;
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        cmd.param1 = 1.0f;
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        cmd.from_external = true;
        command_pub_->publish(cmd);
        RCLCPP_INFO(get_logger(), "Arm command sent");
    }

    void enable_offboard() {
        px4_msgs::msg::VehicleCommand cmd;
        cmd.timestamp = now().nanoseconds() / 1000;
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        cmd.param1 = 1.0f;
        cmd.param2 = 6.0f;  // offboard mode
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        cmd.from_external = true;
        command_pub_->publish(cmd);
        RCLCPP_INFO(get_logger(), "Offboard mode command sent");
    }

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_ = 0;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardController>());
    rclcpp::shutdown();
}