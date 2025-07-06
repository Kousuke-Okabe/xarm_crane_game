#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

// #define GAIN_X -1
// #define GAIN_Y -1
// #define GAIN_Z 1
// #define GAIN_Z_ANGULAR -5
// #define TWIST_SL 0.15
// #define GAIN_JOG -1


class JoystickCraneNode : public rclcpp::Node{
    public:
        JoystickCraneNode() : Node("joy_to_servo_node") {
            twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
                "/servo_server/delta_twist_cmds", 10
            );

            joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
                "/servo_server/delta_joint_cmds", 10
            );
        
            joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy", 10, std::bind(&JoystickCraneNode::subscribe_joy, this, std::placeholders::_1)
            );

            timer_ = this->create_wall_timer(
                100ms, std::bind(&JoystickCraneNode::timer_callback, this)
            );

            timer_param = this->create_wall_timer(
                10s, std::bind(&JoystickCraneNode::timer_param_callback, this)
            );

            this->declare_parameter<double>("GAIN_X", -1.0);
            this->declare_parameter<double>("GAIN_Y", -1.0);
            this->declare_parameter<double>("GAIN_Z", 1.0);
            this->declare_parameter<double>("GAIN_Z_ANGULAR", -5.0);
            this->declare_parameter<double>("TWIST_SL", 0.15);
            this->declare_parameter<double>("GAIN_JOG", -1.0);

            // Request to start_servo
            servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_server/start_servo");
            servo_start_client_->wait_for_service(std::chrono::seconds(1));
            servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        }

    private:
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
        rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer_param;

        geometry_msgs::msg::TwistStamped twist_cmd_;
        control_msgs::msg::JointJog joint_cmd_;
        // auto twist_cmd_ = std::make_unique<geometry_msgs::msg::TwistStamped>();
        // auto joint_cmd_ = std::make_unique<control_msgs::msg::JointJog>();
        double GAIN_X, GAIN_Y, GAIN_Z, GAIN_Z_ANGULAR, TWIST_SL, GAIN_JOG;

        void subscribe_joy(const sensor_msgs::msg::Joy::SharedPtr msg) {
            // Move Twist
            twist_cmd_.header.stamp = this->now();
            twist_cmd_.header.frame_id = "link_base";

            twist_cmd_.twist.linear.x = msg->axes[1] * GAIN_X;
            twist_cmd_.twist.linear.y = msg->axes[0] * GAIN_Y;
            twist_cmd_.twist.linear.z = msg->axes[4] * GAIN_Z;
            twist_cmd_.twist.angular.z = (msg->axes[2] - msg->axes[5]) * GAIN_Z_ANGULAR;

            // Soft limit
            if(twist_cmd_.twist.linear.x < TWIST_SL && twist_cmd_.twist.linear.x > -TWIST_SL)
                twist_cmd_.twist.linear.x = 0.0;
            if(twist_cmd_.twist.linear.y < TWIST_SL && twist_cmd_.twist.linear.y > -TWIST_SL)
                twist_cmd_.twist.linear.y = 0.0;
            if(twist_cmd_.twist.linear.z < TWIST_SL && twist_cmd_.twist.linear.z > -TWIST_SL)
                twist_cmd_.twist.linear.z = 0.0;
            if(twist_cmd_.twist.angular.z < TWIST_SL && twist_cmd_.twist.angular.z > -TWIST_SL)
                twist_cmd_.twist.angular.z = 0.0;

            // // Move Joint
            // if(msg->buttons[3] == 1){
            //     joint_cmd_.header.stamp = this->now();
            //     joint_cmd_.header.frame_id = "joint";
            //     joint_cmd_.joint_names = {"joint2", "joint3", "joint5"};

            //     joint_cmd_.velocities.resize(3);
            //     joint_cmd_.velocities[0] = msg->axes[7] * GAIN_JOG;
            //     joint_cmd_.velocities[1] = msg->axes[7] * GAIN_JOG;
            //     joint_cmd_.velocities[2] = msg->axes[7] * GAIN_JOG;

            //     joint_pub_->publish(joint_cmd_);
            //     // joint_pub_->publish(std::move(joint_cmd_));
            // }

        }

        void timer_callback() {
            twist_pub_->publish(twist_cmd_);
            // twist_cmd_->publish(std::move(twist_cmd_));
        }

        void timer_param_callback() {
            GAIN_X = this->get_parameter("GAIN_X").as_double();
            GAIN_Y = this->get_parameter("GAIN_Y").as_double();
            GAIN_Z = this->get_parameter("GAIN_Z").as_double();
            GAIN_Z_ANGULAR = this->get_parameter("GAIN_Z_ANGULAR").as_double();
            TWIST_SL = this->get_parameter("TWIST_SL").as_double();
            GAIN_JOG = this->get_parameter("GAIN_JOG").as_double();
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickCraneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}