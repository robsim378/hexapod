#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "hexapod_interfaces/msg/leg_position.hpp"

using std::placeholders::_1;

/*
This node is responsible for controlling the servo motors in an individual leg. It 
receives input from that leg's movement controller and moves the servo motors accordingly.
Input is received on a topic, so there is no reporting back to the leg movement controller.
That is done by a separate node that reads the positions of the legs using sensors to more 
accurately reflect the robot's state.
*/
class LegServoController : public rclcpp::Node
{
    public:
        LegServoController()
        : Node("leg_servo_controller")
        {
            // Parameter used to identify which leg this controller is responsible for. Set to an invalid value by default.
            this->declare_parameter("leg_id", -1);

            try 
            {
                // Check if the leg_id has been set. If not, log an error and throw an exception.
                if(this->get_parameter("leg_id").as_int() < 0) 
                {
                    throw std::invalid_argument("Leg movement controller configuration invalid: No leg_id set.");
                }
                // Subscribe to the topic on which to receive commands.
                subscription_ = this->create_subscription<hexapod_interfaces::msg::LegPosition>(
                    "leg_target_position", 10, std::bind(&LegServoController::topic_callback, this, _1));
            }
            
            catch(int e)
            {
                RCLCPP_ERROR(this->get_logger(), "Leg movement controller configuration invalid: No leg_id set.");
            }
        }

    private:
        // Code to execute when receiving a command.
        void topic_callback(const hexapod_interfaces::msg::LegPosition & msg) const
        {
            RCLCPP_INFO(this->get_logger(), "Recieved movement command for leg %li:\njoint1: %f\njoint2: %f\njoint3: %f", this->get_parameter("leg_id").as_int(), msg.joint1, msg.joint2, msg.joint3);
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
