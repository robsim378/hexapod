#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "hexapod_interfaces/msg/leg_position.hpp"

using std::placeholders::_1;

class LegServoController : public rclcpp::Node
{
    public:
        LegServoController()
        : Node("leg_servo_controller")
        {
            subscription_ = this->create_subscription<hexapod_interfaces::msg::LegPosition>(
                "placeholder_topic_name", 10, std::bind(&LegServoController::topic_callback, this, _1));
        }

    private:
        void topic_callback(const hexapod_interfaces::msg::LegPosition & msg) const
        {
            RCLCPP_INFO(this->get_logger(), "Recieved movement command:\njoint1: %f\njoint2: %f\njoint3: %f", msg.joint1, msg.joint2, msg.joint3);
        }
        rclcpp::Subscription<hexapod_interfaces::msg::LegPosition>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LegServoController>());
    rclcpp::shutdown();
    return 0;
}
