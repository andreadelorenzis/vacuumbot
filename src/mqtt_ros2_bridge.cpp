#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <mqtt/async_client.h>
#include "nlohmann/json.hpp"
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include "vacuum_bot/msg/coverage_zone.hpp"
#include "vacuum_bot/msg/coverage_plan.hpp"
#include "vacuum_bot/msg/coverage_mission.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using json = nlohmann::json;

const std::string MQTT_BROKER = "tcp://localhost:1883";
const std::string MQTT_SUB_TOPIC = "mqtt/in";
const std::string MQTT_PUB_TOPIC = "mqtt/out";
const std::string ROS2_SUB_TOPIC = "ros2_in";
const std::string ROS2_PUB_TOPIC = "ros2_out";

class MqttRos2Bridge : public rclcpp::Node, public virtual mqtt::callback {
public:
    MqttRos2Bridge() : Node("mqtt_ros2_bridge"),
        mqtt_client_(MQTT_BROKER, "ros2_client")
    {

        this->declare_parameter<std::string>("ip", "localhost");
        this->declare_parameter<int>("port", 1883);
        auto ip = this->get_parameter("ip").as_string();
        auto port = this->get_parameter("port").as_int();

        // Setup MQTT
        RCLCPP_INFO(this->get_logger(), "Connecting to MSQTT broker at %s:%ld",
            ip.c_str(), port);
        mqtt_client_.set_callback(*this);
        mqtt::connect_options connOpts;
        connOpts.set_keep_alive_interval(20);
        connOpts.set_clean_session(true);
        try {
            mqtt_client_.connect(connOpts)->wait();
            RCLCPP_INFO(this->get_logger(), "Connesso al broker MQTT");
            mqtt_client_.subscribe(MQTT_SUB_TOPIC, 1);
            RCLCPP_INFO(this->get_logger(), "Sottoscritto al topic MQTT %s", MQTT_SUB_TOPIC.c_str());
        } catch (const mqtt::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Errore connessione MQTT: %s", e.what());
        }

        coverage_pub_ = this->create_publisher<geometry_msgs::msg::Polygon>(
            "/coverage_start", 
            10);
        RCLCPP_INFO(this->get_logger(), "Publisher on /coverage_start created");

        manual_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/manual_cmd", 
            10);
        RCLCPP_INFO(this->get_logger(), "Publisher on /manual_cmd created");

        mission_pub_ = this->create_publisher<vacuum_bot::msg::CoverageMission>(
            "/cleaning_mission", 
            10);
        RCLCPP_INFO(this->get_logger(), "Publisher on /cleaning_mission created");

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            10,
            std::bind(&MqttRos2Bridge::map_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to topic /map");

        debug_pub_ = this->create_publisher<std_msgs::msg::String>("/debug", 10);
        publishDebug("MqttRos2Bridge initialized");

        // ROS2 Publisher e Subscriber
        publisher_ = this->create_publisher<std_msgs::msg::String>(ROS2_PUB_TOPIC, 10);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            ROS2_SUB_TOPIC,
            10,
            std::bind(&MqttRos2Bridge::ros2_callback, this, std::placeholders::_1)
        );
    }

    ~MqttRos2Bridge() {
        try {
            mqtt_client_.disconnect()->wait();
        } catch (...) {}
    }

    // ROS2 -> MQTT
    void ros2_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "ROS2 -> MQTT: %s", msg->data.c_str());
        publishDebug("ROS2 -> MQTT: " + msg->data);
        mqtt_client_.publish(MQTT_PUB_TOPIC, msg->data.c_str(), msg->data.size());
    }

    // MQTT Callbacks
    void connected(const std::string&) {}
    void connection_lost(const std::string &cause) {
        RCLCPP_WARN(this->get_logger(), "Connessione MQTT persa: %s", cause.c_str());
    }

    void message_arrived(mqtt::const_message_ptr msg) override {
        std::string payload = msg->get_payload_str();
        RCLCPP_INFO(this->get_logger(), "MQTT -> ROS2: %s", payload.c_str());
        publishDebug("MQTT -> ROS2: " + payload);

        try {
            auto j = json::parse(payload);

            // Coverage
            if (j.contains("type") && j["type"] == "coverage" && j.contains("points")) {
                geometry_msgs::msg::Polygon poly;
                for (auto &pt : j["points"]) {
                    geometry_msgs::msg::Point32 p;
                    p.x = std::stof(pt["x"].get<std::string>());
                    p.y = std::stof(pt["y"].get<std::string>());
                    p.z = 0.0;
                    poly.points.push_back(p);
                }
                if (!poly.points.empty()) {
                    coverage_pub_->publish(poly);
                    RCLCPP_INFO(this->get_logger(), "Published coverage polygon with %zu points", poly.points.size());
                }
            }

            // Manual command
            else if (j.contains("type") && j["type"] == "manual" && j.contains("command")) {
                std::string command = j["command"].get<std::string>();
                std_msgs::msg::String msg;
                msg.data = command;
                manual_pub_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "Published command: %s", command.c_str());
            }

            // Cleaning mission
            else if (j.contains("type") && j["type"] == "plan" && j.contains("zones") && j.contains("ros_base_pos")) {
                vacuum_bot::msg::CoverageMission mission_msg;

                for (auto &zone : j["zones"]) {
                    vacuum_bot::msg::CoverageZone z;
                    z.zone_id = zone["zone_id"].get<int>();
                    z.intensity = zone["intensity"].get<int>();
                    z.frequency = zone["frequency"].get<int>();

                    geometry_msgs::msg::Polygon poly;
                    for (auto &pt : zone["ros_points"]) {
                        geometry_msgs::msg::Point32 p;
                        p.x = pt["x"].get<float>();
                        p.y = pt["y"].get<float>();
                        p.z = 0.0;
                        poly.points.push_back(p);
                    }
                    z.polygon = poly;
                    mission_msg.plan.zones.push_back(z);
                }

                if (j["ros_base_pos"].contains("x") && j["ros_base_pos"].contains("y")) {
                    geometry_msgs::msg::PoseStamped base_pose;
                    base_pose.header.frame_id = "map";
                    base_pose.header.stamp = this->now();
                    base_pose.pose.position.x = j["ros_base_pos"]["x"].get<float>();
                    base_pose.pose.position.y = j["ros_base_pos"]["y"].get<float>();
                    base_pose.pose.position.z = 0.0;
                    base_pose.pose.orientation.w = 1.0; // identità
                    mission_msg.base_pose = base_pose;
                }

                mission_pub_->publish(mission_msg);
                RCLCPP_INFO(this->get_logger(), "Published cleaning mission with %zu zones and base pose", mission_msg.plan.zones.size());
            }

        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse MQTT JSON: %s", e.what());
        }
    }

    void delivery_complete(mqtt::delivery_token_ptr) {}

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        try {
            publishDebug("Message received on topic /map");

            json j;
            j["header"]["stamp"] = msg->header.stamp.sec;
            j["header"]["frame_id"] = msg->header.frame_id;
            j["info"]["resolution"] = msg->info.resolution;
            j["info"]["width"] = msg->info.width;
            j["info"]["height"] = msg->info.height;
            j["info"]["origin"]["position"]["x"] = msg->info.origin.position.x;
            j["info"]["origin"]["position"]["y"] = msg->info.origin.position.y;
            j["info"]["origin"]["position"]["z"] = msg->info.origin.position.z;
            j["info"]["origin"]["orientation"]["x"] = msg->info.origin.orientation.x;
            j["info"]["origin"]["orientation"]["y"] = msg->info.origin.orientation.y;
            j["info"]["origin"]["orientation"]["z"] = msg->info.origin.orientation.z;
            j["info"]["origin"]["orientation"]["w"] = msg->info.origin.orientation.w;
    
            j["data"] = msg->data;
    
            std::string payload = j.dump();
            mqtt_client_.publish(MQTT_PUB_TOPIC, payload.c_str(), payload.size());
            RCLCPP_INFO(this->get_logger(), "Mappa pubblicata su MQTT con %dx%d celle",
                        msg->info.width, msg->info.height);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Errore nella serializzazione della mappa: %s", e.what());
        }
    }

private:
    void publishDebug(const std::string & msg, const std::string & caller = "MqttRos2Bridge") {
        std_msgs::msg::String debug_msg;
        debug_msg.data = "[" + caller + "] " + msg;
        debug_pub_->publish(debug_msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr coverage_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr manual_pub_;
    rclcpp::Publisher<vacuum_bot::msg::CoverageMission>::SharedPtr mission_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
    mqtt::async_client mqtt_client_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MqttRos2Bridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
