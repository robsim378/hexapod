
#include <memory>

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
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
class LegStateBroadcaster: public rclcpp::Node
{
    public:
        LegStateBroadcaster()
        : Node("leg_state_broadcaster")
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
                
                legname_ = "leg_" << this->get_parameter("leg_id").c_str();
                tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

                std::ostringstream stream;
                stream << "/" << legname_.c_str() << "/leg_target_position";
                std::string topic_name = stream.str();

                // Subscribe to the topic on which to receive commands.
                subscription_ = this->create_subscription<hexapod_interfaces::msg::LegPosition>(
                    topic_name, 10, std::bind(&LegServoController::broadcast_position, this, _1));
            }
            
            catch(int e)
            {
                RCLCPP_ERROR(this->get_logger(), "Leg state broadcaster configuration invalid: No leg_id set.");
            }
        }

    private:
        // Code to execute when receiving a command.
        void broadcast_position(const hexapod_interfaces::msg::LegPosition & msg) const
        {
            geometry_msgs::msg::TransformStamped t;

            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "world";
            t.child_frame_id = legname_.c_str();

            // Leg joints can only rotate, not translate.
            t.transform.translation.x = 0.0;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.0;

            tf2::Quaternion q;
            q.setRPY()
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
