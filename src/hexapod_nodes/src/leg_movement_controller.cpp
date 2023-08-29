#include <functional>
#include <memory>
#include <thread>
#include <chrono>
#include <string>
#include <cmath>

#include "hexapod_interfaces/action/leg_movement_command.hpp"
#include "hexapod_interfaces/msg/leg_position.hpp"
#include "hexapod_interfaces/msg/leg_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#define SEGMENT_0_LENGTH 0.5
#define SEGMENT_1_LENGTH 2
#define SEGMENT_2_LENGTH 2

// Max speed of the servo motors, measured in RPM
#define JOINT_MAX_RPM 3000
// Max speed of the servo motors in radians per second
#define JOINT_MAX_SPEED JOINT_MAX_RPM / 50 * 2 * M_PI

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
  // Target is the input received from the main movement controller.
  using Target = hexapod_interfaces::action::LegMovementCommand;
  using GoalHandleTarget = rclcpp_action::ServerGoalHandle<Target>;

  explicit LegMovementController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("leg_movement_controller", options)
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

      // Define the action server for movement commands
      this->action_server_ = rclcpp_action::create_server<Target>(
        this,
        "move_leg",
        std::bind(&LegMovementController::handle_goal, this, _1, _2),
        std::bind(&LegMovementController::handle_cancel, this, _1),
        std::bind(&LegMovementController::handle_accepted, this, _1));

      
      // Set the starting position of the legs
      current_position = inverse_kinematics(0.0, 2.0, -1.0);
      state = forward_kinematics(current_position.joint0, current_position.joint1, current_position.joint2);
      // Publish the calculated position to the leg_n/leg_state topic
      RCLCPP_INFO(this->get_logger(), "Current leg state:\nx_position: %lf\ny_position: %lf\nz_position: %lf\n", state.x_position, state.y_position, state.z_position);
      state_publisher_->publish(state);
    }
    catch(int e)
    {
        RCLCPP_ERROR(this->get_logger(), "Leg movement controller configuration invalid: No leg_id set.");
    }
    
  }

private:
  // Given the angles of all the joints in the leg, 
  // calculate the position of the foot relative to the base of the leg
  hexapod_interfaces::msg::LegState forward_kinematics(float joint0, float joint1, float joint2)
  {
    hexapod_interfaces::msg::LegState result;
    // Calculate the height of the foot
    result.z_position = SEGMENT_1_LENGTH * std::sin(joint1) - SEGMENT_2_LENGTH * std::cos((M_PI - joint2) - (M_PI / 2 - joint1));
    // Calculate the distance of the foot from the joint in the XY plane, ignoring the height
    float current_distance = SEGMENT_1_LENGTH * std::cos(joint1) + SEGMENT_2_LENGTH * std::sin((M_PI - joint2) - (M_PI / 2 - joint1)) + SEGMENT_0_LENGTH;
    // Calculate the X and Y coordinates of the foot
    result.x_position = current_distance * std::sin(joint0);
    result.y_position = current_distance * std::cos(joint0);

    return result;
  }

  // Given the desired position of the foot relative to the base of the leg, 
  // calculate the angles required for all the joints to place the foot there
  hexapod_interfaces::msg::LegPosition inverse_kinematics(float x_position, float y_position, float z_position)
  {
    hexapod_interfaces::msg::LegPosition result;

    // Calculate angle of first joint
    result.joint0 = std::atan(x_position / y_position);

    // Calculate target distance from origin on the XY plane, ignoring the height 
    float target_distance = std::sqrt(std::pow(x_position, 2) + std::pow(y_position, 2)) - SEGMENT_0_LENGTH;
    
    // Inverse kinematics to calculate the positions of the remaining two joints.
    float q2 = -std::acos((std::pow(target_distance, 2) + std::pow(z_position, 2) - std::pow(SEGMENT_1_LENGTH, 2) - std::pow(SEGMENT_2_LENGTH, 2)) / (2 * SEGMENT_1_LENGTH * SEGMENT_2_LENGTH));
    float q1 = std::atan(z_position / target_distance) + std::atan((SEGMENT_2_LENGTH * std::sin(q2)) / (SEGMENT_1_LENGTH + SEGMENT_2_LENGTH * std::cos(q2)));
    result.joint1 = -q1;
    // This calculates the angle that the leg needs to be lowered at, which should be negative by the convention this robot uses for joint angles.
    result.joint2 = -q2;

    return result;
  }

  // Get the current movement command, calculate the current position of the leg, and send the appropriate
  // information to the servo controller and the main movement controller.
  void loop_timer_callback()
  {
    state = forward_kinematics(current_position.joint0, current_position.joint1, current_position.joint2);
    // Publish the calculated position to the leg_n/leg_state topic
    RCLCPP_INFO(this->get_logger(), "Current leg state:\nx_position: %lf\ny_position: %lf\nz_position: %lf\n", state.x_position, state.y_position, state.z_position);
    state_publisher_->publish(state);
  }

  // The code to execute when a goal is received
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Target::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received movement command:\ngrounded: %i\nx_position: %lf\ny_position: %lf\nz_position: %lf", goal->grounded, goal->x_position, goal->y_position, goal->z_position);
    // command.grounded = goal->grounded;
    // command.x_position = goal->x_position;
    // command.y_position = goal->y_position;
    // command.z_position = -goal->z_position;

    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // The code to execute when a cancel request is received for a goal
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTarget> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  
  // The code to execute when accepting a goal request
  void handle_accepted(const std::shared_ptr<GoalHandleTarget> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&LegMovementController::execute, this, _1), goal_handle}.detach();
  }

  // The code to execute when working towards a goal
  void execute(const std::shared_ptr<GoalHandleTarget> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    
    
    rclcpp::Rate loop_rate(1);
    
    const auto goal = goal_handle->get_goal();
    
    auto feedback = std::make_shared<Target::Feedback>();
    
    auto result = std::make_shared<Target::Result>();


    // Determine the target position of the movement
    // Inverse kinematics: 
    target_position = inverse_kinematics(goal->x_position, goal->y_position, -goal->z_position);



    for (int i = 0; i < 10 && rclcpp::ok() && result->success == false; i++) {
      
      

      // Calculate the next position of the leg, then send it to the servo motor.

      // First, calculate the next state to put the leg into. Then, perform inverse kinematics to determine the position
      // of all the joints, then publish that on the command topic.
    
      // Log the results
      RCLCPP_INFO(this->get_logger(), "Sending command to leg %li:\njoint0: %lf\njoint1: %lf\njoint2: %lf", this->get_parameter("leg_id").as_int(), target_position.joint0, target_position.joint1, target_position.joint2);

      // Send the movement command to the leg servo controller on the leg_n/leg_target_position topic
      command_publisher_->publish(target_position);

      loop_rate.sleep();

      // Get the current position of the leg from sensors
      // tf2 listener goes here

      current_position.joint0 = target_position.joint0;
      current_position.joint1 = target_position.joint1;
      current_position.joint2 = target_position.joint2;

      // Calculate the position of the leg in the LegState format, then send it to the main movement controller
      // Forward kinematics: 
      state = forward_kinematics(current_position.joint0, current_position.joint1, current_position.joint2);
      // Publish the calculated position to the leg_n/leg_state topic
      RCLCPP_INFO(this->get_logger(), "Current leg state:\nx_position: %lf\ny_position: %lf\nz_position: %lf\n", state.x_position, state.y_position, state.z_position);
      state_publisher_->publish(state);

      RCLCPP_INFO(this->get_logger(), "state/goal diff:\nx_position: %lf\ny_position: %lf\nz_position: %lf\n", std::abs(state.x_position - goal->x_position), std::abs(state.y_position - goal->y_position), std::abs(state.z_position - goal->z_position));

      if (std::abs(state.x_position - goal->x_position) <= 0.1 && std::abs(state.y_position - goal->y_position) <= 0.1 && std::abs(state.z_position - goal->z_position) <= 0.1) {
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      }
    }



    
  }
  rclcpp_action::Server<Target>::SharedPtr action_server_;

  rclcpp::TimerBase::SharedPtr loop_timer_;
  rclcpp::Publisher<hexapod_interfaces::msg::LegPosition>::SharedPtr command_publisher_;
  rclcpp::Publisher<hexapod_interfaces::msg::LegState>::SharedPtr state_publisher_;
  rclcpp::Subscription<hexapod_interfaces::action::LegMovementCommand>::SharedPtr command_subscriber_;

  hexapod_interfaces::msg::LegPosition target_position;
  hexapod_interfaces::msg::LegPosition current_position;
  hexapod_interfaces::action::LegMovementCommand command;
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