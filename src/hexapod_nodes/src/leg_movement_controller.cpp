#include <functional>
#include <memory>
#include <thread>

#include "hexapod_interfaces/action/leg_movement_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class LegMovementController : public rclcpp::Node
{
public:
  using Target = hexapod_interfaces::action::LegMovementCommand;
  using GoalHandleTarget = rclcpp_action::ServerGoalHandle<Target>;

  //HEXAPOD_NODES_PUBLIC
  explicit LegMovementController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("leg_movement_controller", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Target>(
      this,
      "move_leg",
      std::bind(&LegMovementController::handle_goal, this, _1, _2),
      std::bind(&LegMovementController::handle_cancel, this, _1),
      std::bind(&LegMovementController::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Target>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Target::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received movement request:\nangle: %f\nh_position: %f\nv_position: %f\n", goal->angle, goal->h_position, goal->v_position);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTarget> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleTarget> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&LegMovementController::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleTarget> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Target::Result>();

    loop_rate.sleep();

	//result->success = true
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    
  }
};  // class LegMovementController 

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto leg_movement_controller = std::make_shared<LegMovementController>();

  rclcpp::spin(leg_movement_controller);

  rclcpp::shutdown();
  return 0;
}