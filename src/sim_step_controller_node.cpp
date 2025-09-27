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

#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>


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

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        last_odom_time_ = this->now();

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

        publish_odom();
    }

    void publish_odom() 
    {
        // Delta t
        auto now_time = this->now();
        double dt = (now_time - last_odom_time_).seconds();
        last_odom_time_ = now_time;

        // Current velocity
        double linear = command_map.at(current_state).first;
        double angular = command_map.at(current_state).second;

        // Pose integration
        double delta_x = linear * cos(theta_) * dt;
        double delta_y = linear * sin(theta_) * dt;
        double delta_theta = angular * dt;

        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;

        while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
        while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

        // Publish odometry message
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = now_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.twist.twist.linear.x = linear;
        odom_msg.twist.twist.angular.z = angular;

        odom_pub_->publish(odom_msg);

        // Publish TF odom → base_link
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = now_time;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_link";
        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation.x = q.x();
        odom_tf.transform.rotation.y = q.y();
        odom_tf.transform.rotation.z = q.z();
        odom_tf.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(odom_tf);
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

    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
    rclcpp::Time last_odom_time_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

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

