#include <thread>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <memory>

#include "hexapod_interfaces/action/leg_motion_command.hpp"
#include "hexapod_interfaces/action/leg_step_command.hpp"
#include "hexapod_interfaces/msg/foot_position.hpp"
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


            foot_position_subscriber_ = this->create_subscription<hexapod_interfaces::msg::FootPosition>(
                "foot_position",
                10,
                std::bind(&LegStepController::foot_position_callback, this, _1));

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

        RCLCPP_INFO(this->get_logger(), "Sending motion goal");

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
            // RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
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
        motion_command_complete = 1;
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Motion goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Motion goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
        RCLCPP_INFO(this->get_logger(), "Motion goal completed");
    }

    // ########## FOOT POSITION CALLBACK ##########

    // Function called when the position of a foot is received
    void foot_position_callback(const hexapod_interfaces::msg::FootPosition & position_msg) 
    {
        foot_position.x_position = position_msg.x_position;
        foot_position.y_position = position_msg.y_position;
        foot_position.z_position = position_msg.z_position;
    }


    // ########## STEP COMMAND CALLBACKS ##########

    // The function to execute when a goal is received
    rclcpp_action::GoalResponse handle_step_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const StepTarget::Goal> goal)
    {
        // For now just accept all commands
        RCLCPP_INFO(this->get_logger(), "Received step command:\ngrounded: %i\nspeed: %lf\nx_position: %lf\ny_position: %lf\nz_position: %lf", goal->grounded, goal->speed, goal->x_position, goal->y_position, goal->z_position);
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

    // This function calculates the height at a given position x along the parabola with the equation
    // y = 0.5(1/n)(n-x)(x), where n is the distance between the two x-intercepts.
    // This parabola maintains the same shape scaled up or down depending on the value of n.
    float calculate_height_on_parabola(float n, float x) {
        return 0.5 * (1 / n) * (n - x) * x;
    }

    // The function to execute when working towards a goal
    void execute(const std::shared_ptr<GoalHandleStepTarget> goal_handle)
    {
        // RCLCPP_INFO(this->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<StepTarget::Feedback>();
        auto result = std::make_shared<StepTarget::Result>();

        auto motion_goal = MotionTarget::Goal();




        // Step planning 


        // The number of subdivisions of the movement (i.e. how many intermediate positions are passed through on the way to the target)
        // Fewer subdivisions for faster movement.
        int num_divisions = 10;

        // Variable used to hold the value of how much the foot should be raised by
        float added_height = 0;
        float new_height = 0;
        float last_height = 0;

        // If goal->grounded is set to false, the foot will trace a parabola between its starting and ending points in order to not
        // drag along the ground. This is done by adding the y value of the curve the z_position of the foot for the distance travelled
        // (e.g. halfway through the step, it reaches the vertex)

        // The distances covered by the foot in all 3 directions
        float x_distance = (goal->x_position - foot_position.x_position);
        float y_distance = (goal->y_position - foot_position.y_position);
        float z_distance = (goal->z_position - foot_position.z_position);


        // The total distance covered by the foot
        float total_distance = std::sqrt(std::pow(x_distance, 2) + std::pow(y_distance, 2) + std::pow(z_distance, 2));

        RCLCPP_INFO(this->get_logger(), "Distance to cover in this step:\nx: %lf\ny: %lf\nz: %lf\ntotal: %lf\n", x_distance, y_distance, z_distance, total_distance);
        

        // Calculate how far the foot must move in each direction per subdivision
        float x_distance_per_division = x_distance / num_divisions;
        float y_distance_per_division = y_distance / num_divisions;
        float z_distance_per_division = z_distance / num_divisions;


        // The total distance the foot moves per subdivision
        float total_distance_per_division = std::sqrt(std::pow(x_distance_per_division, 2) + std::pow(y_distance_per_division, 2) + std::pow(z_distance_per_division, 2));

        RCLCPP_INFO(this->get_logger(), "Distance to cover in this step per subdivision:\nx: %lf\ny: %lf\nz: %lf\ntotal: %lf\n", x_distance_per_division, y_distance_per_division, z_distance_per_division, total_distance_per_division);

        // The frequency with which to check if the foot has finished its subdivision
        rclcpp::Rate motion_command_check_rate(10);

        // Move the legs through each subdivision. For each iteration, calculate the joint angles for the next position using
        // inverse kinematics, then publish that position on the command topic.
        for (int i = 0; i < num_divisions && rclcpp::ok() && result->success == false; i++) {
            
            if (!goal->grounded) {
                last_height = new_height;
                new_height = calculate_height_on_parabola(total_distance, (i + 1) * total_distance_per_division);
                added_height = new_height - last_height;
            }

            // Calculate the next position of the foot along the path
            motion_goal.speed = goal->speed;
            motion_goal.x_position = foot_position.x_position + x_distance_per_division;
            motion_goal.y_position = foot_position.y_position + y_distance_per_division;
            motion_goal.z_position = foot_position.z_position + z_distance_per_division + added_height;
            
            send_motion_goal(motion_goal);

            // Wait for the motion command to be completed. 
            while (!motion_command_complete) 
            {
                motion_command_check_rate.sleep();
            }

            motion_command_complete = 0;
        }    
        RCLCPP_INFO(this->get_logger(), "current_x_position - goal_x_position diff: %lf\ncurrent_y_position - goal_y_position diff: %lf\ncurrent_z_position - goal_z_position diff: %lf\n", std::abs(foot_position.x_position - goal->x_position), std::abs(foot_position.y_position - goal->y_position), std::abs(foot_position.z_position - goal->z_position));
        // Check if the final position has been reached.
        if (std::abs(foot_position.x_position - goal->x_position) <= 0.01 && std::abs(foot_position.y_position - goal->y_position) <= 0.01 && std::abs(foot_position.z_position - goal->z_position) <= 0.01) 
        {
            result->success = true;
            goal_handle->succeed(result);
            // RCLCPP_INFO(this->get_logger(), "Step goal succeeded");
        }
        else 
        {
            goal_handle->abort(result);
            // RCLCPP_INFO(this->get_logger(), "Step goal aborted");
        }
        



    }


    // ########## DECLARATIONS ##########

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Client<MotionTarget>::SharedPtr leg_motion_command_client_ptr_;
    rclcpp_action::Server<StepTarget>::SharedPtr leg_step_command_server_;


    rclcpp::Subscription<hexapod_interfaces::msg::FootPosition>::SharedPtr foot_position_subscriber_;
    hexapod_interfaces::msg::FootPosition foot_position;
    hexapod_interfaces::action::LegMotionCommand command;
    char motion_command_complete = 1;

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