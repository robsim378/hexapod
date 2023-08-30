#include <thread>
#include <chrono>

#include "hexapod_interfaces/leg_motion_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

/*
This class is responsible for receiving a command from the gait controller to step in a specific direction
and calculating the path the foot should take, then sending commands to the leg motion controller to move along that path. 
*/
class LegStepController : public rclcpp::Node
{
public:
    using MotionTarget = hexapod_interfaces::action::LegMotionCommand;
    using GoalHandleMotionTarget = rclcpp_action::ClientGoalHandle<MotionTarget>;

    explicit LegStepController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("leg_step_controller", options)
    {
        // Create the timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&LegStepController::send_motion_goal,
            this));
        
        // Create action client for sending motion commands to the leg motion controller
        this->leg_motion_command_client_ptr_ = rclcpp_action::create_client<MotionTarget>(
            this,
            "move_leg");
    }

    // Send a motion command to the leg motion controller
    void send_motion_goal()
    {
        using namespace std::placeholders;

        // Validate that an action server is available
        if (!this->leg_motion_command_client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = MotionTarget::Goal();

        // Placeholder command
        goal_msg.speed = 0.50;
        goal_msg.x_position = 0.0;
        goal_msg.y_position = 1.0;
        goal_msg.z_position = -1.0;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_motion_goal_options = rclcpp_action::Client<MotionTarget>::SendGoalOptions();
        send_motion_goal_options.goal_response_callback = 
            std::bind(&LegStepController::goal_response_callback, this, _1);
        send_motion_goal_options.feedback_callback =
            std::bind(&LegStepController::feedback_callback, this, _1, _2);
        send_motion_goal_options.result_callback = 
            std::bind(&LegStepController::result_callback, this, _1);
        this->leg_motion_command_client_ptr_->async_send_goal(goal_msg, send_motion_goal_options);
    }


private:
    // Function called when a response is received from an action
    void goal_response_callback(const GoalHandleMotionTarget::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }
    
    // Function called when feedback is received from an action
    void feedback_callback(
        GoalHandleMotionTarget::SharedPtr, 
        const std::shared_ptr<const MotionTarget::Feedback> feedback)
    {

    }

    
    // Function called when a result is received from an action
    void result_callback(const GoalHandleMotionTarget::WrappedResult & result)
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


    // Declarations
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Client<MotionTarget>::SharedPtr leg_motion_command_client_ptr_;

}   // class LegStepController

// Main method. Self explanatory.
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto leg_step_controller = std::make_shared<LegStepController>(); 
    rclcpp::spin(leg_step_controller);
    rclcpp::shutdown();
    return 0;
}