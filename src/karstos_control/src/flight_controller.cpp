#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <actuator_msgs/msg/actuators.hpp>
#include <Eigen/Dense>
#include <cmath>

// PID Controller section
struct PID {
    double kp, ki, kd;
    double i_max, out_max;
    double integral = 0;
    double prev_error = 0;
    double prev_t = -1;

    PID(double kp, double ki, double kd,
        double i_max=2.0, double out_max=1e9)
      : kp(kp), ki(ki), kd(kd),
        i_max(i_max), out_max(out_max) {}

    double update(double error, double t) {
        if (prev_t < 0) {
            prev_t = t;    // just record time, return 0
            prev_error = error;
            return 0.0;    // ← return 0 not kp*error
        }
        double dt = t - prev_t;
        if (dt <= 0) return 0;
        integral = std::clamp(integral + error*dt, -i_max, i_max);
        double deriv = (error - prev_error) / dt;
        prev_error = error; prev_t = t;
        return std::clamp(
            kp*error + ki*integral + kd*deriv,
            -out_max, out_max);
    }
};

// PD Controller
struct PD {
    double kp, kd, out_max;
    PD(double kp, double kd, double out_max=1e9)
      : kp(kp), kd(kd), out_max(out_max) {}
    double update(double error, double derror) {
        return std::clamp(kp*error + kd*derror, -out_max, out_max);
    }
};

// Motor mixer
// ─────────────────────────────────────────────────────────
//  Motor Mixer — X configuration quadrotor
//
//    Motor 0 (CCW)  Motor 1 (CW)
//          \          /
//           [  drone  ]
//          /          
//    Motor 3 (CW)   Motor 2 (CCW)
//
//  Input:  [thrust, roll, pitch, yaw]
//  Output: [ω0, ω1, ω2, ω3] in rad/s
//
//  Allocation matrix — each row is one motor:
//    +thrust always positive (lifts)
//    +roll   → tilt right  (motor 0,3 faster)
//    +pitch  → tilt forward (motor 0,1 faster)
//    +yaw    → rotate CW   (CW motors faster)

Eigen::Vector4d mix(double thrust, double roll,
                    double pitch,  double yaw) {
    Eigen::Matrix4d A;
    A << 1, -1,  1,  1,   // motor 0: front-right CCW
        1, -1, -1, -1,   // motor 1: rear-left   CCW
        1,  1,  1, -1,   // motor 2: front-left  CW
        1,  1, -1,  1;   // motor 3: rear-right  CW

    Eigen::Vector4d cmd(thrust, roll, pitch, yaw);
    Eigen::Vector4d w2 = A * cmd;

    const double wmin = 400.0, wmax = 1500.0;
    Eigen::Vector4d w;
    for (int i = 0; i < 4; i++) {
        w[i] = std::sqrt(std::max(w2[i], wmin*wmin));
        w[i] = std::clamp(w[i], wmin, wmax);
    }
    return w;
}

// Flight controller node
class FligthController : public rclcpp::Node {
    public:
        FligthController() : Node("flight_controller"),
            // Outer loops
            pd_x_(0.03, 0.015),
            pd_y_(0.03, 0.015),
            pid_z_(0.8, 0.05, 2.0, 1.5, 3.0),

            // Increase attitude gains — faster correction
            pid_roll_(15.0, 0.5, 4.0, 0.5, 1.5),
            pid_pitch_(15.0, 0.5, 4.0, 0.5, 1.5),
            pid_yaw_(5.0, 0.1, 2.0, 0.3, 0.5)
        {

            this->set_parameter(rclcpp::Parameter("use_sim_time", true));

            // Subscribers
            imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "/drone/imu",
                10,
                [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                imu_cb(msg); });

            pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
                "/drone/pose_raw", 10,
                [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    double t = msg->header.stamp.sec +
                            msg->header.stamp.nanosec * 1e-9;
                    double px = msg->pose.position.x;
                    double py = msg->pose.position.y;
                    double pz = msg->pose.position.z;

                    if (prev_pose_t_ > 0) {
                        double dt = t - prev_pose_t_;
                        if (dt > 0 && dt < 1.0) {
                            vel_x_ = (px - pos_x_) / dt;
                            vel_y_ = (py - pos_y_) / dt;
                            vel_z_ = (pz - pos_z_) / dt;
                        }
                    }
                    pos_x_ = px; pos_y_ = py; pos_z_ = pz;
                    prev_pose_t_ = t;
                });

            // Publisher
            motor_pub_ = create_publisher<actuator_msgs::msg::Actuators>(
            "/drone/motors", 10);

            // Controll loop at 100 hz
            timer_ = create_wall_timer(
                std::chrono::milliseconds(10),
                [this]() { control_loop(); });

            arm_timer_ = create_wall_timer(
                std::chrono::seconds(5),
                [this]() {
                    armed_ = true;
                    arm_timer_->cancel();
                    RCLCPP_INFO(get_logger(), "Armed");
                });

            RCLCPP_INFO(get_logger(), "Flight controller is ready");
        }
    private:
        void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) {
            // Quaternion to Euler
            double x = msg->orientation.x;
            double y = msg->orientation.y;
            double z = msg->orientation.z;
            double w = msg->orientation.w;

            // Check for the uninitialized quaternion
            if (x*x + y*y + z*z + w*w < 0.1) return;

            // Roll (rotation around X)
            double sr = 2*(w*x + y*z);
            double cr = 1 - 2*(x*x + y*y);
            roll_ = std::atan2(sr, cr);

            // Pitch (rotation around Y)
            double sp = 2*(w*y - z*x);
            pitch_ = std::asin(std::clamp(sp, -1.0, 1.0));

            // Yaw (rotation around Z)
            double sy = 2*(w*z + x*y);
            double cy = 1 - 2*(y*y + z*z);
            yaw_ = std::atan2(sy, cy);
        }

        // ── Pose callback: position from SLAM ───────────────
        void pose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
        {
            // t must be defined here — before anything uses it
            double t = msg->header.stamp.sec +
                    msg->header.stamp.nanosec * 1e-9;

            double px = msg->pose.pose.position.x;
            double py = msg->pose.pose.position.y;
            double pz = msg->pose.pose.position.z;

            if (prev_pose_t_ > 0) {
                double dt = t - prev_pose_t_;
                if (dt > 0 && dt < 1.0) {
                    vel_x_ = (px - pos_x_) / dt;
                    vel_y_ = (py - pos_y_) / dt;
                    vel_z_ = (pz - pos_z_) / dt;
                }
            }

            pos_x_ = px; pos_y_ = py; pos_z_ = pz;
            prev_pose_t_ = t;
            last_pose_time_ = t;  // ← now t exists here
        }

        void control_loop()
        {
            if (!armed_) { publish_motors({0,0,0,0}); return; }
            if (prev_pose_t_ < 0) { publish_motors({0,0,0,0}); return; }

            // Use wall clock for PID dt, pose for position values
            auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
            double t = now_ns * 1e-9;
            
            double z_err = sp_z_ - pos_z_;
            double z_damp = -0.8 * vel_z_;

            double thrust = mass_ * g_ + pid_z_.update(z_err, t) + z_damp;
            double des_pitch = pd_x_.update(sp_x_ - pos_x_, -vel_x_);
            double des_roll  = pd_y_.update(sp_y_ - pos_y_, -vel_y_);

            double max_tilt = 15.0 * M_PI / 180.0;
            des_pitch = std::clamp( des_pitch, -max_tilt, max_tilt);
            des_roll  = std::clamp( des_roll,  -max_tilt, max_tilt);

            double rc = pid_roll_.update( des_roll  - roll_,  t);
            double pc = pid_pitch_.update(des_pitch - pitch_, t);

            double yaw_err = sp_yaw_ - yaw_;
            yaw_err = std::fmod(yaw_err + M_PI, 2*M_PI) - M_PI;
            double yc = pid_yaw_.update(yaw_err, t);

            double k = 8.54858e-6;
            Eigen::Vector4d motors = mix(
                thrust / (4*k),
                rc * 800,
                pc * 800,
                yc * 500
            );

            publish_motors({motors[0], motors[1], motors[2], motors[3]});

            if (++log_n_ % 200 == 0) {
                RCLCPP_INFO(get_logger(),
                    "pos=(%.2f,%.2f,%.2f) sp=(%.1f,%.1f,%.1f) "
                    "rpy=(%.1f,%.1f,%.1f)deg "
                    "des_rp=(%.1f,%.1f)deg "
                    "rc=%.3f pc=%.3f "
                    "motors=[%.0f,%.0f,%.0f,%.0f]",
                    pos_x_, pos_y_, pos_z_,
                    sp_x_,  sp_y_,  sp_z_,
                    roll_*180/M_PI, pitch_*180/M_PI, yaw_*180/M_PI,
                    des_roll*180/M_PI, des_pitch*180/M_PI,
                    rc, pc,
                    motors[0], motors[1], motors[2], motors[3]);
            }
        }

        void publish_motors(std::vector<double> v) {
            auto msg = actuator_msgs::msg::Actuators();
            msg.velocity = v;
            motor_pub_->publish(msg);
        }

        //controllers
        PD  pd_x_, pd_y_;
        PID pid_z_, pid_roll_, pid_pitch_, pid_yaw_;

        // ── State ──
        double roll_=0, pitch_=0, yaw_=0;
        double pos_x_=0, pos_y_=0, pos_z_=0;
        double vel_x_=0, vel_y_=0, vel_z_=0;
        double prev_pose_t_ = -1;

        // ── Setpoint ──
        double sp_x_=.0, sp_y_=0, sp_z_=3.0, sp_yaw_=0;

        // ── Physical constants ──
        const double mass_ = 1.5;
        const double g_    = 9.81;

        bool   armed_ = false;
        int    log_n_ = 0;
        double last_pose_time_ = -1;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr motor_pub_;
        rclcpp::TimerBase::SharedPtr timer_, arm_timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FligthController>());
    rclcpp::shutdown();
    return 0;
}