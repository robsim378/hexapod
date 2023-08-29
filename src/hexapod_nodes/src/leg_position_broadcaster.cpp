#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <math.h>

#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hexapod_interfaces/msg/joint_angles.hpp"

using std::placeholders::_1;

/*
This node is responsible for broadcasting the current state of its leg on the tf2 topic.
*/
class LegPositionBroadcaster: public rclcpp::Node
{
    public:
        LegPositionBroadcaster()
        : Node("leg_position_broadcaster")
        {
            // Parameter used to identify which leg this broadcaster is responsible for. Set to an invalid value by default.
            this->declare_parameter("leg_id", -1);
            
            try 
            {
                // Check if the leg_id has been set. If not, log an error and throw an exception.
                if(this->get_parameter("leg_id").as_int() < 0) 
                {
                    throw std::invalid_argument("Leg state broadcaster configuration invalid: No leg_id set.");
                }
                leg_id = this->get_parameter("leg_id").as_int();

                legname_ = "leg_" + std::to_string(leg_id);

                // Subscribe to the topic on which to receive commands.
                subscription_ = this->create_subscription<hexapod_interfaces::msg::JointAngles>(
                    "target_joint_angles", 10, std::bind(&LegPositionBroadcaster::broadcast_position, this, _1));

                publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
                    "/joint_states",
                    10);
            }
            
            catch(int e)
            {
                RCLCPP_ERROR(this->get_logger(), "Leg state broadcaster configuration invalid: No leg_id set.");
            }
        }

    private:
        // Code to execute when receiving a command.
        void broadcast_position(const std::shared_ptr<hexapod_interfaces::msg::JointAngles> msg) 
        {
            // Define the JointState messages for the three joints in the leg
            sensor_msgs::msg::JointState joint_angles;

            joint_angles.header.stamp = this->get_clock()->now();
            joint_angles.name.resize(3);
            joint_angles.position.resize(3);

            // Set the names of the three joints to match the URDF file
            joint_angles.name[0] = legname_ + "_body_to_segment_0";
            joint_angles.name[1] = legname_ + "_segment_0_to_segment_1";
            joint_angles.name[2] = legname_ + "_segment_1_to_segment_2";

            // Set the positions of the three joints
            joint_angles.position[0] = msg->joint0;
            joint_angles.position[1] = -msg->joint1;
            joint_angles.position[2] = msg->joint2;

            RCLCPP_INFO(this->get_logger(), "Recieved motion command for leg %li:\njoint0: %f\njoint1: %f\njoint2: %f", this->get_parameter("leg_id").as_int(), msg->joint0, msg->joint1, msg->joint2);
            publisher_->publish(joint_angles);

        }
        int leg_id;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
        rclcpp::Subscription<hexapod_interfaces::msg::JointAngles>::SharedPtr subscription_;
        std::string legname_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LegPositionBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
