#ifndef MAIN_CONTROL_HPP
#define MAIN_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "RobStrideMotor.hpp"
#include "CanTransport.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <memory>

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

    void handle_read_packet();
    void handle_write_packet();
    void transition_to(ControlState new_state);
    void initParameters();

    // 멤버 변수
    std::vector<CanBusGroup> can_groups_;
    std::vector<std::shared_ptr<RobStrideMotor>> all_motors_;  // 모든 CAN의 모터를 순서대로 보관
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr walk_sub;

    ControlState current_state;

    std::vector<MotorCommand> motor_commands_;
    std::mutex command_mutex_;
};

#endif // MAIN_CONTROL_HPP
