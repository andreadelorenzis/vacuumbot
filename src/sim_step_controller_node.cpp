#include <chrono>
#include <memory>
#include <string>
#include <assert.h>
#include <stdio.h>
#include <mutex>
#include <deque>
#include <thread>
#include <chrono>
#include <atomic>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "easywsclient.hpp"
#include <arpa/inet.h>
#include <unistd.h>

using std::placeholders::_1;

enum class RobotState {
    STOP,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

class StepController : public rclcpp::Node
{
public:
    StepController(const rclcpp::NodeOptions & options)
    : Node("step_controller", options)
    {
        clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

        RCLCPP_INFO(this->get_logger(), "Running in simulation mode");
        pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/diff_cont/cmd_vel", 
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
        RCLCPP_INFO(this->get_logger(), "Publisher on /diff_cont/cmd_vel created");

        subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel_nav", rclcpp::QoS(10).reliable(),
        std::bind(&StepController::cmd_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Subscription on /cmd_vel created!");

        manual_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/manual_cmd", 10, std::bind(&StepController::manual_cmd_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Subscription on /manual_cmd created!");
            
        // Publisher per debug
        debug_pub_ = this->create_publisher<std_msgs::msg::String>("/debug", 10);

        // Subscription al topic /cmd_intensity
        intensity_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/cmd_intensity", 10,
            std::bind(&StepController::intensityCallback, this, std::placeholders::_1)
        );

        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&StepController::send_command, this));

        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&StepController::check_watchdog, this));

        last_cmd_time_ = now();
        current_state = RobotState::STOP;

        publishDebug("SimStepController initialized!");
    }

    private:

    void publishDebug(const std::string & msg) {
        std_msgs::msg::String debug_msg;
        debug_msg.data = msg;
        debug_pub_->publish(debug_msg);
    }

    void intensityCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        int intensity = msg->data;

        std_msgs::msg::String debug_msg;
        publishDebug("Changed intensity to " + std::to_string(intensity));

        RCLCPP_INFO(this->get_logger(), "Changed intensity to value %d", intensity);
    }

    void cmd_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        last_cmd_time_ = now();
        const double EPS = 1e-3;
        RobotState new_state = RobotState::STOP;

        double abs_linear  = std::abs(msg->twist.linear.x);
        double abs_angular = std::abs(msg->twist.angular.z);

        if (std::abs(msg->twist.linear.x) > EPS &&  abs_linear > abs_angular) {
            new_state = (msg->twist.linear.x > 0) ? RobotState::FORWARD : RobotState::BACKWARD;
        } else if (std::abs(msg->twist.angular.z) > EPS && abs_linear <= abs_angular) {
            new_state = (msg->twist.angular.z > 0) ? RobotState::LEFT : RobotState::RIGHT;
        }

        if (new_state != current_state) {
            current_state = new_state;
            RCLCPP_INFO(this->get_logger(), "Current state: %s", state_to_string(current_state).c_str());
        }
    }

    void manual_cmd_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string cmd = msg->data;
        if (cmd == "LEFT")       current_state = RobotState::LEFT;
        else if (cmd == "RIGHT") current_state = RobotState::RIGHT;
        else if (cmd == "FORWARD") current_state = RobotState::FORWARD;
        else if (cmd == "STOP")  current_state = RobotState::STOP;

        // Set last time two seconds in the future so that the watchdog is
        // not immediately triggered
        auto now_time = now();              
        last_cmd_time_ = now_time + std::chrono::seconds(2); 
    }

    void send_command()
    {
        auto it = command_map.find(this->current_state);
        auto msg = geometry_msgs::msg::TwistStamped();
        msg.header.stamp = this->now(); 
        msg.header.frame_id = "base_link";
        msg.twist.linear.x  = it->second.first;
        msg.twist.angular.z = it->second.second;
        pub_->publish(msg);
    }

    void check_watchdog()
    {
        auto now_time = now();
        auto dt = (now_time - last_cmd_time_).seconds();
        if (dt > 0.2 && current_state != RobotState::STOP) { // 200ms timeout
            current_state = RobotState::STOP;
            RCLCPP_WARN(this->get_logger(), "Watchdog triggered: stopping robot");
        }
    }


    std::string state_to_string(RobotState state) {
        switch (state) {
            case RobotState::STOP:     return "STOP";
            case RobotState::FORWARD:  return "FORWARD";
            case RobotState::BACKWARD: return "BACKWARD";
            case RobotState::LEFT:     return "LEFT";
            case RobotState::RIGHT:    return "RIGHT";
        }
        return "UNKNOWN";
    }


    const std::map<RobotState, std::pair<double,double>> command_map = {
        {RobotState::FORWARD,  {0.5,  0.0}},
        {RobotState::BACKWARD, {-0.5, 0.0}},
        {RobotState::LEFT,     {0.0,  0.8}},
        {RobotState::RIGHT,    {0.0, -0.8}},
        {RobotState::STOP,     {0.0,  0.0}}
    };

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr intensity_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr manual_cmd_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    rclcpp::Clock::SharedPtr clock_;
    RobotState current_state;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::Time last_cmd_time_;
};
  
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("use_sim_time", true)
    });
    auto node = std::make_shared<StepController>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

