#include <thread>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <memory>

#include "hexapod_interfaces/action/leg_motion_command.hpp"
#include "hexapod_interfaces/action/leg_step_command.hpp"
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

    using StepTarget = hexapod_interfaces::action::LegStepCommand;
    using GoalHandleStepTarget = rclcpp_action::ServerGoalHandle<StepTarget>;

    explicit LegStepController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("leg_step_controller", options)
    {
        using namespace std::placeholders;

        // Parameter used to identify which leg this controller is used for. Set to an invalid value by default.
        this->declare_parameter("leg_id", -1);
        
        try 
        {
            // Check if the leg_id has been set. If not, log an error and throw an exception.
            if(this->get_parameter("leg_id").as_int() < 0) 
            {
                throw std::invalid_argument("Leg movement controller configuration invalid: No leg_id set.");
            }

            RCLCPP_INFO(this->get_logger(), "Starting leg_%li step controller", this->get_parameter("leg_id").as_int());

            // // Create the timer
            // timer_ = this->create_wall_timer(
            //     std::chrono::milliseconds(500),
            //     std::bind(&LegStepController::send_motion_goal,
            //     this));
        
            // Create action client for sending motion commands to the leg motion controller
            this->leg_motion_command_client_ptr_ = rclcpp_action::create_client<MotionTarget>(
                this,
                "move_leg");

            // Create action server for handling step commands
            this->leg_step_command_server_ = rclcpp_action::create_server<StepTarget>(
                this,
                "step",
                std::bind(&LegStepController::handle_step_goal, this, _1, _2),
                std::bind(&LegStepController::handle_step_cancel, this, _1),
                std::bind(&LegStepController::handle_step_accepted, this, _1));

        }
        catch(int e)
        {
            RCLCPP_ERROR(this->get_logger(), "Leg step controller configuration invalid: No leg_id set.");
        }
        
    }

    // Send a motion command to the leg motion controller
    void send_motion_goal(MotionTarget::Goal goal_msg)
    {
        using namespace std::placeholders;

        // Validate that an action server is available
        if (!this->leg_motion_command_client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        // auto goal_msg = MotionTarget::Goal();

        // Placeholder command
        // goal_msg.speed = 0.50;
        // goal_msg.x_position = 0.0;
        // goal_msg.y_position = 1.0;
        // goal_msg.z_position = -1.0;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_motion_goal_options = rclcpp_action::Client<MotionTarget>::SendGoalOptions();
        send_motion_goal_options.goal_response_callback = 
            std::bind(&LegStepController::motion_goal_response_callback, this, _1);
        send_motion_goal_options.feedback_callback =
            std::bind(&LegStepController::motion_feedback_callback, this, _1, _2);
        send_motion_goal_options.result_callback = 
            std::bind(&LegStepController::motion_result_callback, this, _1);
        this->leg_motion_command_client_ptr_->async_send_goal(goal_msg, send_motion_goal_options);
    }


private:

    // ########## MOTION COMMAND CALLBACKS ##########

    // Function called when a response is received from an action
    void motion_goal_response_callback(const GoalHandleMotionTarget::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }
    
    // Function called when feedback is received from an action
    void motion_feedback_callback(
        GoalHandleMotionTarget::SharedPtr, 
        const std::shared_ptr<const MotionTarget::Feedback> feedback)
    {

    }

    
    // Function called when a result is received from an action
    void motion_result_callback(const GoalHandleMotionTarget::WrappedResult & result)
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


    // ########## STEP COMMAND CALLBACKS ##########

    // The function to execute when a goal is received
    rclcpp_action::GoalResponse handle_step_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const StepTarget::Goal> goal)
    {
        // For now just accept all commands
        RCLCPP_INFO(this->get_logger(), "Received step command:\ngronded: %i\nspeed: %lf\nx_position: %lf\ny_position: %lf\nz_position: %lf", goal->grounded, goal->speed, goal->x_position, goal->y_position, goal->z_position);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // The function to execute when a cancel request is received for a goal
    rclcpp_action::CancelResponse handle_step_cancel(
        const std::shared_ptr<GoalHandleStepTarget> goal_handle)
    {
        // For now just accept all cancellations
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
  
    // The function to execute when accepting a goal request
    void handle_step_accepted(const std::shared_ptr<GoalHandleStepTarget> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&LegStepController::execute, this, _1), goal_handle}.detach();
    }

    // The function to execute when working towards a goal
    void execute(const std::shared_ptr<GoalHandleStepTarget> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<StepTarget::Feedback>();
        auto result = std::make_shared<StepTarget::Result>();

        // Step planning goes here, for now just echo 
        auto goal_msg = MotionTarget::Goal();
        goal_msg.speed = goal->speed;
        goal_msg.x_position = goal->x_position;
        goal_msg.y_position = goal->y_position;
        goal_msg.z_position = goal->z_position;

        send_motion_goal(goal_msg);
    }


    // ########## DECLARATIONS ##########

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Client<MotionTarget>::SharedPtr leg_motion_command_client_ptr_;
    rclcpp_action::Server<StepTarget>::SharedPtr leg_step_command_server_;

    hexapod_interfaces::action::LegMotionCommand command;

};   // class LegStepController

// Main method. Self explanatory.
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto leg_step_controller = std::make_shared<LegStepController>(); 
    rclcpp::spin(leg_step_controller);
    rclcpp::shutdown();
    return 0;
}