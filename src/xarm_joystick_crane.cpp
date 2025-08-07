#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <hand_control_interfaces/msg/move_hand.hpp>

using namespace std::chrono_literals;

class JoystickCraneNode : public rclcpp::Node{
    public:
        JoystickCraneNode() : Node("joy_to_servo_node") {
            twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
                "/servo_server/delta_twist_cmds", 10
            );

            joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
                "/servo_server/delta_joint_cmds", 10
            );
            
            hand_pub_ = this->create_publisher<hand_control_interfaces::msg::MoveHand>(
                "/hand_control", 10
            );
        
            joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy", 10, std::bind(&JoystickCraneNode::subscribe_joy, this, std::placeholders::_1)
            );

            timer_ = this->create_wall_timer(
                10ms, std::bind(&JoystickCraneNode::timer_callback, this)
            );

            timer_param = this->create_wall_timer(
                5s, std::bind(&JoystickCraneNode::timer_param_callback, this)
            );

            this->declare_parameter<double>("GAIN_X", -1.0);
            this->declare_parameter<double>("GAIN_Y", -1.0);
            this->declare_parameter<double>("GAIN_Z", 1.0);
            this->declare_parameter<double>("GAIN_Z_ANGULAR", -5.0);
            this->declare_parameter<double>("TWIST_SL", 0.15);
            this->declare_parameter<int>("Hand_ID", 1);

            // Request to start_servo
            servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_server/start_servo");
            servo_start_client_->wait_for_service(std::chrono::seconds(1));
            servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        }

    private:
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
        rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
        
        rclcpp::Publisher<hand_control_interfaces::msg::MoveHand>::SharedPtr hand_pub_;

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer_param;

        geometry_msgs::msg::TwistStamped twist_cmd_;
        hand_control_interfaces::msg::MoveHand hand_cmd_;
        double GAIN_X, GAIN_Y, GAIN_Z, GAIN_Z_ANGULAR, TWIST_SL;
        int HAND_ID;

        void subscribe_joy(const sensor_msgs::msg::Joy::SharedPtr msg) {
            // Move Twist
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

            // Move Hand
            if(msg->buttons[0] == 1) { // Button A
                hand_cmd_.id = HAND_ID;
                hand_cmd_.state = 'C'; // Open
            }
            else if(msg->buttons[1] == 1) { // Button B
                hand_cmd_.id = HAND_ID;
                hand_cmd_.state = 'O'; // Close
            }    
        }

        void timer_callback() {
            twist_cmd_.header.stamp = this->now();
            
            twist_pub_->publish(twist_cmd_);
            hand_pub_->publish(hand_cmd_);
            // twist_cmd_->publish(std::move(twist_cmd_));
        }

        void timer_param_callback() {
            GAIN_X = this->get_parameter("GAIN_X").as_double();
            GAIN_Y = this->get_parameter("GAIN_Y").as_double();
            GAIN_Z = this->get_parameter("GAIN_Z").as_double();
            GAIN_Z_ANGULAR = this->get_parameter("GAIN_Z_ANGULAR").as_double();
            TWIST_SL = this->get_parameter("TWIST_SL").as_double();
            HAND_ID = this->get_parameter("Hand_ID").as_int();
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickCraneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}