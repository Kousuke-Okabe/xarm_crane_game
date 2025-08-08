#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <hand_control_interfaces/msg/move_hand.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>

#define al_1    0
#define al_2    -M_PI / 2
#define al_3    0
#define al_4    -M_PI / 2
#define al_5     M_PI / 2
#define al_6    -M_PI / 2

#define off_1   0
#define off_2   -1.3849179
#define off_3    1.3849179
#define off_4   0.0
#define off_5   0.0
#define off_6   0.0

using namespace std::chrono_literals;

class JoystickCraneNode : public rclcpp::Node{
    public:
        JoystickCraneNode() : Node("joy_to_servo_node") {
            twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
                "/servo_server/delta_twist_cmds", 10
            );

            // joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
            //     "/servo_server/delta_joint_cmds", 10
            // );
            
            hand_pub_ = this->create_publisher<hand_control_interfaces::msg::MoveHand>(
                "/hand_control", 10
            );
        
            joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy", 10, std::bind(&JoystickCraneNode::subscribe_joy, this, std::placeholders::_1)
            );

            jointState_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&JoystickCraneNode::subscribe_jointState, this, std::placeholders::_1)
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
        // rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
        
        rclcpp::Publisher<hand_control_interfaces::msg::MoveHand>::SharedPtr hand_pub_;

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointState_sub_;

        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer_param;

        geometry_msgs::msg::TwistStamped twist_cmd_;
        hand_control_interfaces::msg::MoveHand hand_cmd_;
        double GAIN_X, GAIN_Y, GAIN_Z, GAIN_Z_ANGULAR, TWIST_SL;
        int HAND_ID;

        double al[6] = {al_1, al_2, al_3, al_4, al_5, al_6};
        double off[6] = {off_1, off_2, off_3, off_4, off_5, off_6};
        double joint_pos[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        tf2::Quaternion quaternion_ee;
        tf2::Quaternion quaternion_ref;
        tf2::Quaternion quaternion_sub;

        void subscribe_joy(const sensor_msgs::msg::Joy::SharedPtr msg) {
            quaternion_ref.setW(0.0);
            quaternion_ref.setX(-1.0);
            quaternion_ref.setY(0.0);
            quaternion_ref.setZ(0.0);

            // Move Twist
            twist_cmd_.header.frame_id = "link_base";

            twist_cmd_.twist.linear.x = msg->axes[1] * GAIN_X;
            twist_cmd_.twist.linear.y = msg->axes[0] * GAIN_Y;
            twist_cmd_.twist.linear.z = msg->axes[4] * GAIN_Z;
            twist_cmd_.twist.angular.z = (msg->axes[2] - msg->axes[5]) * GAIN_Z_ANGULAR;

            // Control orientation
            quaternion_sub = quaternion_ref * quaternion_ee.inverse();
            twist_cmd_.twist.angular.x = GAIN_Z_ANGULAR*( quaternion_sub.getX() );
            twist_cmd_.twist.angular.y = GAIN_Z_ANGULAR*( quaternion_sub.getY() );


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

        void subscribe_jointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
            // Get joint positions
            for(int i = 0; i < 6; i++) {
                joint_pos[i] = msg->position[i];
            }

            // Calculate the quaternions for the end effector
            quaternion_ee.setW(1.0);
            quaternion_ee.setX(0.0);
            quaternion_ee.setY(0.0);
            quaternion_ee.setZ(0.0);

            for(int i = 0; i < 6; i++){
                tf2::Quaternion qua_a, qua_o, qua_th;

                qua_a.setW( cos(al[i] / 2) );
                qua_a.setX( sin(al[i] / 2) );
                qua_a.setY( 0.0 );
                qua_a.setZ( 0.0 );

                qua_o.setW( cos(off[i] / 2) );
                qua_o.setX( 0.0 );
                qua_o.setY( 0.0 );
                qua_o.setZ( sin(off[i] / 2) );

                qua_th.setW( cos(joint_pos[i] / 2) );
                qua_th.setX( 0.0 );
                qua_th.setY( 0.0 );
                qua_th.setZ( sin(joint_pos[i] / 2) );

                quaternion_ee = qua_th * qua_o * qua_a * quaternion_ee;
            }

            // RCLCPP_INFO(this->get_logger(), "Quaternion EE: [%f, %f, %f, %f]",
            //     quaternion_ee.getX(), quaternion_ee.getY(), quaternion_ee.getZ(), quaternion_ee.getW());
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