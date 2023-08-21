#include <functional>
#include <memory>
#include <thread>
#include <chrono>

#include "hexapod_interfaces/action/leg_movement_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

/*
This node is responsible for controlling the movement of the entire hexapod. It determines 
which legs to move and where to move them based on the current status of the robot and the
input received. It then sends commands to each leg's leg movement controller to handle 
actually moving the legs.
*/
class MainMovementController : public rclcpp::Node
{
public:
    using Target = hexapod_interfaces::action::LegMovementCommand;
    using GoalHandleTarget = rclcpp_action::ClientGoalHandle<Target>;

    explicit MainMovementController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("main_movement_controller", options)
    {
        // create action client for an individual leg
        this->client_ptr_ = rclcpp_action::create_client<Target>(
            this,
            "leg_4/move_leg");
        
        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MainMovementController::send_goal, 
            this));
    }

    void send_goal()
    {
        using namespace std::placeholders;

        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = Target::Goal();
        goal_msg.angle = 0.0;
        goal_msg.h_position = 0.785;
        goal_msg.v_position = -1.571;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<Target>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&MainMovementController::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&MainMovementController::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = 
            std::bind(&MainMovementController::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<Target>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void goal_response_callback(const GoalHandleTarget::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleTarget::SharedPtr, 
        const std::shared_ptr<const Target::Feedback> feedback)
    {
        
    }

    void result_callback(const GoalHandleTarget::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
        RCLCPP_INFO(this->get_logger(), "Step completed");
    }
};

// Main method. Self explanatory.
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto main_movement_controller = std::make_shared<MainMovementController>();

  rclcpp::spin(main_movement_controller);

  rclcpp::shutdown();
  return 0;
}