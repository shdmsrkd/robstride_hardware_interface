#ifndef MAIN_CONTROL_HPP
#define MAIN_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "RobStrideMotor.hpp"
#include "CanTransport.hpp"
#include "filter.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include <memory>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <chrono>

enum class ControlState
{
    WRITE_PACKET,
    READ_PACKET
};

struct MotorCommand {
    float position;
    float velocity;
    float torque;
    float kp;
    float kd;
};

class MainControlNode : public rclcpp_lifecycle::LifecycleNode {
public:
    // 생성자 및 소멸자
    explicit MainControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~MainControlNode();

    // Lifecycle 콜백 함수들
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &);

private:
    // 주기적으로 실행될 제어 루프
    void control_loop();
    void walkCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void torqueCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void handle_read_packet();
    void handle_write_packet();
    void transition_to(ControlState new_state);
    void initParameters();

    void toCSV(float pos, float vel);

    void canSetup();
    std::string execute_command(const std::string& cmd);

    // 멤버 변수
    std::vector<CanBusGroup> can_groups_;
    std::vector<std::shared_ptr<RobStrideMotor>> all_motors_;  // 모든 CAN의 모터를 순서대로 보관
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr walk_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr torque_sub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr state_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr initial_pub;

    ControlState current_state;

    std::vector<MotorCommand> motor_commands_;
    std::mutex command_mutex_;

    std::vector<Butterworth2ndOrderLPF> velocity_filters_;
    std::chrono::steady_clock::time_point last_velocity_filter_time_;
    bool velocity_filter_time_initialized_ = false;

    // 초기 set 자세
    bool walk_initialized_ = false;
    bool start_positions_captured_ = false;
    std::vector<float> start_positions_;
    int init_tick_count_ = 0;
    static constexpr int INIT_TOTAL_TICKS = 100;
};

#endif // MAIN_CONTROL_HPP
