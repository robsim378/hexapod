#include <functional>
#include <memory>
#include <thread>
#include <chrono>
#include <string>
#include <cmath>

#include "hexapod_interfaces/msg/leg_movement_command.hpp"
#include "hexapod_interfaces/msg/leg_position.hpp"
#include "hexapod_interfaces/msg/leg_state.hpp"
#include "rclcpp/rclcpp.hpp"

#define SEGMENT_0_LENGTH 0.5
#define SEGMENT_1_LENGTH 2
#define SEGMENT_2_LENGTH 2

using namespace std::chrono_literals;

/*
This node is responsible for handling the pathfinding and control of an individual leg.
It receives input in the form of a target destination for the foot, and it calculates a 
path and sends commands to the corresponding servo controller to move. This is the node 
that handles inverse kinematics.
*/
class LegMovementController : public rclcpp::Node
{
public:
  LegMovementController()
  : Node("leg_movement_controller")//, count_(0)
  {
    using namespace std::placeholders;

    // Parameter used to set the frequency of the main loop
    this->declare_parameter("control_frequency", 1);

    // Parameter used to identify which leg this controller is used for. Set to an invalid value by default.
    this->declare_parameter("leg_id", -1);

    try
    {
      // Check if the leg_id has been set. If not, log an error and throw an exception.
      if(this->get_parameter("leg_id").as_int() < 0) 
      {
        throw std::invalid_argument("Leg movement controller configuration invalid: No leg_id set.");
      }

      // Define the publisher for sending commands to the servo controller
      command_publisher_ = this->create_publisher<hexapod_interfaces::msg::LegPosition>(
        "leg_target_position", 10);

      loop_timer_ = this->create_wall_timer(
        std::chrono::seconds((int)(1.0 / this->get_parameter("control_frequency").as_int())), 
        std::bind(&LegMovementController::loop_timer_callback, 
        this));

      // Define the publisher for reporting the state of the leg to the main movement controller
      state_publisher_ = this->create_publisher<hexapod_interfaces::msg::LegState>(
        "leg_state", 10);

      // Define the subscriber for receiving movement commands from the main movement controller
      command_subscriber_ = this->create_subscription<hexapod_interfaces::msg::LegMovementCommand>(
        "leg_movement_command", 10, std::bind(&LegMovementController::command_callback, this, _1));

    }
    catch(int e)
    {
        RCLCPP_ERROR(this->get_logger(), "Leg movement controller configuration invalid: No leg_id set.");
    }
    
  }

private:

  // Get the current movement command, calculate the current position of the leg, and send the appropriate
  // information to the servo controller and the main movement controller.
  void loop_timer_callback()
  {
    // Get the current position of the leg from sensors
    // tf2 listener goes here

    // Calculate the position of the leg in the LegState format, then send it to the main movement controller
    // Forward kinematics goes here, for now it just echoes
    state.x_position = current_position.joint0;
    state.y_position = current_position.joint1;
    state.z_position = current_position.joint2;
    RCLCPP_INFO(this->get_logger(), "Current leg state:\nx_position: %lf\ny_position: %lf\nz_position: %lf", state.x_position, state.y_position, state.z_position);
    state_publisher_->publish(state);

    // Calculate the next position of the leg, then send it to the servo motor.
    // This will become motion planning and inverse kinematics, for now it just echoes.

    // First, calculate the next state to put the leg into. Then, perform inverse kinematics to determine the position
    // of all the joints, then publish that on the command topic.

    // Calculate angle of first joint and put it here
    target_position.joint0 = std::atan(command.x_position / command.y_position);

    // Calculate distance from origin, ignoring the z_position
    float distance = std::sqrt(std::pow(command.x_position, 2) + std::pow(command.y_position, 2)) - SEGMENT_0_LENGTH;
    
    // Inverse kinematics to calculate the positions of the remaining two joints.
    float q2 = -std::acos((std::pow(distance, 2) + std::pow(command.z_position, 2) - std::pow(SEGMENT_1_LENGTH, 2) - std::pow(SEGMENT_2_LENGTH, 2)) / (2 * SEGMENT_1_LENGTH * SEGMENT_2_LENGTH));
    float q1 = std::atan(command.z_position / distance) + std::atan((SEGMENT_2_LENGTH * std::sin(q2)) / (SEGMENT_1_LENGTH + SEGMENT_2_LENGTH * std::cos(q2)));
    target_position.joint1 = -q1;
    // This calculates the angle that the leg needs to be lowered at, which should be negative by the convention this robot uses for joint angles.
    target_position.joint2 = -q2;
    
    RCLCPP_INFO(this->get_logger(), "Sending command to leg %li:\njoint0: %lf\njoint1: %lf\njoint2: %lf", this->get_parameter("leg_id").as_int(), target_position.joint0, target_position.joint1, target_position.joint2);

    // Send the movement command to the leg servo controller
    command_publisher_->publish(target_position);
  }

  // Receive a movement command and update the stored command
  void command_callback(const hexapod_interfaces::msg::LegMovementCommand & msg) 
  {
    RCLCPP_INFO(this->get_logger(), "Received movement command:\ngrounded: %i\nx_position: %lf\ny_position: %lf\nz_position: %lf", msg.grounded, msg.x_position, msg.y_position, msg.z_position);
    command.grounded = msg.grounded;
    command.x_position = msg.x_position;
    command.y_position = msg.y_position;
    command.z_position = -msg.z_position;
  }

  rclcpp::TimerBase::SharedPtr loop_timer_;
  rclcpp::Publisher<hexapod_interfaces::msg::LegPosition>::SharedPtr command_publisher_;
  rclcpp::Publisher<hexapod_interfaces::msg::LegState>::SharedPtr state_publisher_;
  rclcpp::Subscription<hexapod_interfaces::msg::LegMovementCommand>::SharedPtr command_subscriber_;

  hexapod_interfaces::msg::LegPosition target_position;
  hexapod_interfaces::msg::LegPosition current_position;
  hexapod_interfaces::msg::LegMovementCommand command;
  hexapod_interfaces::msg::LegState state;
};  // class LegMovementController 

// Main method. Self explanatory.
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto leg_movement_controller = std::make_shared<LegMovementController>();

  rclcpp::spin(leg_movement_controller);

  rclcpp::shutdown();
  return 0;
}