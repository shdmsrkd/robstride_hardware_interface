/*
TODO: 데이터셋 수집 노드로 퍼블리시 할 목록

1️⃣ 현재 위치 (Position)

    의미: 모터가 지금 실제로 있는 각도

    용도: 목표 위치와의 오차 계산

    이름 예시: motorStatePos

    단위: rad

2️⃣ 현재 속도 (Velocity)

    의미: 모터 회전 속도

    용도: 마찰·감쇠·동특성 반영

    이름 예시: motorStateVel

    단위: rad/s

3️⃣ 목표 위치 (Desired Position / Action)

    의미: 포지션 제어기에 넣은 목표 각도

    용도: 오차(action - pos)의 기준

    이름 예시: motorAction

    단위: rad

4️⃣ 출력 전류 (Current)

    의미: 포지션 제어 결과로 실제 흐른 전류

    용도: 액추에이터 출력(≈토크) 학습 라벨

    이름 예시: motorStateCur

    단위: A (또는 mA, 고정 필요)

*/


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
    READ_PACKET,
    RECEIVE_CMD,
    WRITE_PACKET,
    ERROR
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

    // TODO : 데이터셋 수집 노드로 발행
    void publish_status();
    void handle_write_packet();
    void handle_error();
    void transition_to(ControlState new_state);
    void initParameters();

    // 멤버 변수
    std::shared_ptr<CanTransport> transport_;
    std::vector<std::shared_ptr<RobStrideMotor>> motors_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr walk_sub;

    // TODO : 데이터셋 수집노드로 퍼블리시
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr status_pub;
    ControlState current_state;
    bool receive_flag = false;

    std::vector<MotorCommand> motor_commands_;
    std::mutex command_mutex_;
};

#endif // MAIN_CONTROL_HPP
