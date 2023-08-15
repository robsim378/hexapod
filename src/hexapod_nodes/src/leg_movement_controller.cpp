#include <functional>
#include <memory>
#include <thread>

#include "hexapod_interfaces/action/leg_movement_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

/*
This node is responsible for handling the pathfinding and control of an individual leg.
It receives input in the form of a target destination for the foot, and it calculates a 
path and sends commands to the corresponding servo controller to move.
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

    // Define the action server for movement commands
    this->action_server_ = rclcpp_action::create_server<Target>(
      this,
      "move_leg",
      std::bind(&LegMovementController::handle_goal, this, _1, _2),
      std::bind(&LegMovementController::handle_cancel, this, _1),
      std::bind(&LegMovementController::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Target>::SharedPtr action_server_;

  // The code to execute when a goal is received
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Target::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received movement request:\nangle: %f\nh_position: %f\nv_position: %f\n", goal->angle, goal->h_position, goal->v_position);
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
    auto result = std::make_shared<Target::Result>();

    loop_rate.sleep();

    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    
  }
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