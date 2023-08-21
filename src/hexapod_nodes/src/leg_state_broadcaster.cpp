#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <math.h>

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hexapod_interfaces/msg/leg_position.hpp"

#define BODY_RADIUS 0.5
#define LEG_RADIUS 0.2
#define SEGMENT_0_LENGTH 0.5
#define SEGMENT_1_LENGTH 1
#define SEGMENT_2_LENGTH 2

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
                leg_id = this->get_parameter("leg_id").as_int();

                legname_ = "leg_" + std::to_string(leg_id);
                tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

                std::ostringstream stream;
                stream << "/" << legname_.c_str() << "/leg_target_position";
                std::string topic_name = stream.str();

                // Subscribe to the topic on which to receive commands.
                subscription_ = this->create_subscription<hexapod_interfaces::msg::LegPosition>(
                    topic_name, 10, std::bind(&LegStateBroadcaster::broadcast_position, this, _1));
            }
            
            catch(int e)
            {
                RCLCPP_ERROR(this->get_logger(), "Leg state broadcaster configuration invalid: No leg_id set.");
            }
        }

    private:
        // Code to execute when receiving a command.
        void broadcast_position(const std::shared_ptr<hexapod_interfaces::msg::LegPosition> msg) 
        {
            // Define the transforms for the three joints in the leg
            geometry_msgs::msg::TransformStamped joint0;
            geometry_msgs::msg::TransformStamped joint1;
            geometry_msgs::msg::TransformStamped joint2;

            joint0.header.stamp = this->get_clock()->now();
            joint1.header.stamp = this->get_clock()->now();
            joint2.header.stamp = this->get_clock()->now();

            // Define the parent and child frames of the transform
            joint0.header.frame_id = "body";
            joint0.child_frame_id = legname_ + "_segment_0";

            joint1.header.frame_id = legname_ + "_segment_0";
            joint1.child_frame_id = legname_ + "_segment_1";

            joint2.header.frame_id = legname_ + "_segment_1";
            joint2.child_frame_id = legname_ + "_segment_2";

            // Define the positions of the joints relative to each other
            joint0.transform.translation.x = BODY_RADIUS*sin(leg_id*M_PI/3);
            joint0.transform.translation.y = BODY_RADIUS*cos(leg_id*M_PI/3);
            joint0.transform.translation.z = 0.0;

            joint1.transform.translation.x = 0.0;
            joint1.transform.translation.y = 0.0;
            joint1.transform.translation.z = SEGMENT_0_LENGTH;

            joint2.transform.translation.x = 0.0;
            joint2.transform.translation.y = 0.0;
            joint2.transform.translation.z = SEGMENT_1_LENGTH;

            // Set rotation of joints
            tf2::Quaternion q0;
            q0.setRPY(msg->joint0, M_PI/2, M_PI/2-leg_id*M_PI/3);
            joint0.transform.rotation.x = q0.x();
            joint0.transform.rotation.y = q0.y();
            joint0.transform.rotation.z = q0.z();
            joint0.transform.rotation.w = q0.w();

            tf2::Quaternion q1;
            // The negation here is to establish the convention of 
            // positive angles raising the joint up and negative ones
            // lowering it.
            q1.setRPY(0, -msg->joint1, 0);
            joint1.transform.rotation.x = q1.x();
            joint1.transform.rotation.y = q1.y();
            joint1.transform.rotation.z = q1.z();
            joint1.transform.rotation.w = q1.w();

            tf2::Quaternion q2;
            q2.setRPY(0, -msg->joint2, 0);
            joint2.transform.rotation.x = q2.x();
            joint2.transform.rotation.y = q2.y();
            joint2.transform.rotation.z = q2.z();
            joint2.transform.rotation.w = q2.w();

            // Send the transforms
            tf_broadcaster_->sendTransform(joint0);
            tf_broadcaster_->sendTransform(joint1);
            tf_broadcaster_->sendTransform(joint2);
        }
        int leg_id;
        rclcpp::Subscription<hexapod_interfaces::msg::LegPosition>::SharedPtr subscription_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::string legname_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LegStateBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
