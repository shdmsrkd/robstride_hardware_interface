#include "robstride_rdk_ros2/MainControl.hpp"
#include <cmath>



static float wrapToPi(float angle)
{
    angle = std::fmod(angle, 2.0f * static_cast<float>(M_PI));
    if (angle > static_cast<float>(M_PI))
        angle -= 2.0f * static_cast<float>(M_PI);
    else if (angle < -static_cast<float>(M_PI))
        angle += 2.0f * static_cast<float>(M_PI);
    return angle;
}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

MainControlNode::MainControlNode(const rclcpp::NodeOptions & options)
    : LifecycleNode("main_control_node", options), current_state(ControlState::WRITE_PACKET)
{
    RCLCPP_INFO(this->get_logger(), "MainControlNode created");
}

MainControlNode::~MainControlNode()
{
    RCLCPP_INFO(this->get_logger(), "MainControlNode destroyed");
}

void MainControlNode::initParameters()
{
    declare_parameter("baud_rate", 1000000);
    declare_parameter<std::vector<std::string>>("can_interfaces", {"can0"});

    auto can_interfaces = get_parameter("can_interfaces").as_string_array();

    for (const auto& can_name : can_interfaces)
    {
        declare_parameter<std::vector<int64_t>>(can_name + ".motor_ids", std::vector<int64_t>{});
        declare_parameter<std::vector<int64_t>>(can_name + ".motor_type", std::vector<int64_t>{});
    }

    RCLCPP_INFO(this->get_logger(), "[Configure] Parameters initialized");
    RCLCPP_INFO(this->get_logger(), "[Configure] baud_rate: %ld", get_parameter("baud_rate").as_int());
    RCLCPP_INFO(this->get_logger(), "[Configure] can_interfaces count: %zu", can_interfaces.size());
    for (const auto& can_name : can_interfaces)
    {
        auto ids = get_parameter(can_name + ".motor_ids").as_integer_array();
        auto types = get_parameter(can_name + ".motor_type").as_integer_array();
        RCLCPP_INFO(this->get_logger(), "[Configure] %s: %zu motors", can_name.c_str(), ids.size());
    }
}

std::string MainControlNode::execute_command(const std::string& cmd)
{
    std::array<char, 128> buffer;
    std::string result;

    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);

    if (!pipe) {
      RCLCPP_ERROR(this->get_logger(), "popen() 실패! 명령어를 실행할 수 없습니다.");
      return "";
    }

    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
      result += buffer.data();
    }

    return result;

}

void MainControlNode::canSetup()
{
    execute_command("sudo ip link set can0 down");
    execute_command("sudo ip link set can0 up type can bitrate 1000000");
    execute_command("sleep 0.5");

    execute_command("sudo ip link set can1 down");
    execute_command("sudo ip link set can1 up type can bitrate 1000000");
    execute_command("sleep 0.5");

    execute_command("sudo ip link set can2 down");
    execute_command("sudo ip link set can2 up type can bitrate 1000000");
    execute_command("sleep 0.5");

    execute_command("sudo ip link set can3 down");
    execute_command("sudo ip link set can3 up type can bitrate 1000000");

    std::string result = execute_command("ip link show can0 2>&1");
    if(result.find("state UP") != std::string::npos)
    { RCLCPP_INFO(this->get_logger(), "CAN interface setup successful! 'can0' is activated."); }
    else
    { RCLCPP_ERROR(this->get_logger(), "CAN interface setup failed! 'can0' is not activated."); }

    result = execute_command("ip link show can1 2>&1");
    if(result.find("state UP") != std::string::npos)
    { RCLCPP_INFO(this->get_logger(), "CAN interface setup successful! 'can1' is activated."); }
    else
    { RCLCPP_ERROR(this->get_logger(), "CAN interface setup failed! 'can1' is not activated."); }

    result = execute_command("ip link show can2 2>&1");
    if(result.find("state UP") != std::string::npos)
    { RCLCPP_INFO(this->get_logger(), "CAN interface setup successful! 'can2' is activated."); }
    else
    { RCLCPP_ERROR(this->get_logger(), "CAN interface setup failed! 'can2' is not activated."); }

    result = execute_command("ip link show can3 2>&1");
    if(result.find("state UP") != std::string::npos)
    { RCLCPP_INFO(this->get_logger(), "CAN interface setup successful! 'can3' is activated."); }
    else { RCLCPP_ERROR(this->get_logger(), "CAN interface setup failed! 'can3' is not activated."); }
}

CallbackReturn MainControlNode::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "[Configure] Configuring...");

    canSetup();
    rclcpp::QoS cmd_qos(rclcpp::KeepLast(1));
    cmd_qos.reliable();
    rclcpp::QoS motor_status_qos(rclcpp::KeepLast(1));
    motor_status_qos.best_effort();

    // Publisher 초기화
    state_pub = this->create_publisher<roa_interfaces::msg::MotorStateArray>("/hardware_interface/state", motor_status_qos);
    // state_pub_1 = this->create_publisher<roa_interfaces::msg::MotorStateArray>("/hardware_interface/state_1", motor_status_qos);
    initial_pub = this->create_publisher<std_msgs::msg::Bool>("walk_initialized", cmd_qos);

    walk_sub = this->create_subscription<roa_interfaces::msg::MotorCommandArray>(
        "/hardware_interface/command", cmd_qos,
        std::bind(&MainControlNode::walkCallback, this, std::placeholders::_1)
    );

    torque_sub = this->create_subscription<std_msgs::msg::Bool>(
        "/hardware_interface/etop", 10,
        std::bind(&MainControlNode::torqueCallback, this, std::placeholders::_1)
    );
    // 파라미터 초기화
    initParameters();

    auto can_interfaces = get_parameter("can_interfaces").as_string_array();

    // 각 CAN 인터페이스별 초기화
    can_groups_.clear();
    all_motors_.clear();
    motor_id_to_index_.clear();

    for (const auto& can_name : can_interfaces)
    {
        CanBusGroup group;
        group.interface_name = can_name;
        group.transport = std::make_shared<CanTransport>();

        if (!group.transport->open(can_name))
        {
            RCLCPP_ERROR(this->get_logger(), "[Configure] Failed to open CAN interface '%s'", can_name.c_str());
            return CallbackReturn::FAILURE;
        }

        // 해당 CAN의 motor_ids, motor_type 가져오기
        std::vector<int64_t> motor_ids = get_parameter(can_name + ".motor_ids").as_integer_array();
        std::vector<int64_t> motor_types = get_parameter(can_name + ".motor_type").as_integer_array();

        if (motor_ids.size() != motor_types.size())
        {
            RCLCPP_ERROR(this->get_logger(), "[Configure] %s: motor_ids and motor_type size mismatch", can_name.c_str());
            return CallbackReturn::FAILURE;
        }

        // 모터 객체 초기화
        for (size_t i = 0; i < motor_ids.size(); i++)
        {
            ActuatorType type = static_cast<ActuatorType>(motor_types[i]);
            uint8_t id = static_cast<uint8_t>(motor_ids[i]);

            auto motor = std::make_shared<RobStrideMotor>(group.transport, id, type);
            group.motors.push_back(motor);
            all_motors_.push_back(motor);

            RCLCPP_INFO(this->get_logger(), "[Configure] %s Motor[%zu] initialized - ID: %d, Type: %ld",
                can_name.c_str(), i, id, motor_types[i]);
        }

        can_groups_.push_back(std::move(group));
    }

    // motor ID to packit index mapper
    for (size_t i = 0; i < all_motors_.size(); ++i)
    {
        uint16_t id = static_cast<uint16_t>(all_motors_[i]->getMotorId());

        if (motor_id_to_index_.find(id) != motor_id_to_index_.end())
        {
            RCLCPP_ERROR(this->get_logger(),
                "[Configure] Duplicate motor_id detected: %u", id);
            return CallbackReturn::FAILURE;
        }

        motor_id_to_index_[id] = i;

        RCLCPP_INFO(this->get_logger(),
            "[Configure] Packet mapper: motor_id=%u -> packet_index=%zu",
            id, i);
    }

    // packet 버퍼 초기화
    packet_commands_.commands.resize(all_motors_.size());
    for (size_t i = 0; i < all_motors_.size(); ++i)
    {
        uint16_t id = static_cast<uint16_t>(all_motors_[i]->getMotorId());
        packet_commands_.commands[i].motor_id = id;
        packet_commands_.commands[i].torque   = 0.0f;
        packet_commands_.commands[i].position = 0.0f;
        packet_commands_.commands[i].velocity = 0.0f;
        packet_commands_.commands[i].kp       = 0.0f;
        packet_commands_.commands[i].kd       = 0.0f;
    }

    packet_initialized_ = false;

    RCLCPP_INFO(this->get_logger(),
        "[Configure] Configured successfully with %zu CAN interfaces, %zu total motors",
        can_groups_.size(), all_motors_.size());
    return CallbackReturn::SUCCESS;
}

CallbackReturn MainControlNode::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "[Activate] Activating...");

    // 모든 모터 활성화
    for (size_t i = 0; i < all_motors_.size(); i++)
    {
        if (all_motors_[i]->enable())
        {
            RCLCPP_INFO(this->get_logger(), "[Activate] Motor[%zu] enabled successfully", i);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "[Activate] Failed to enable motor[%zu]", i);
            return CallbackReturn::FAILURE;
        }
    }

    // 제어 루프 타이머 시작 (200Hz, WRITE/READ 번갈아 실행하여 제어주기 100Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&MainControlNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "[Activate] Activated successfully with %zu motors", all_motors_.size());
    return CallbackReturn::SUCCESS;
}

void MainControlNode::control_loop()
{
    // Hz 측정 (WRITE 기준으로 제어주기 측정)
    static auto last_print_time = std::chrono::steady_clock::now();
    static int write_count = 0;

    if (current_state == ControlState::WRITE_PACKET)
    {
        write_count++;
        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = current_time - last_print_time;

        if (elapsed.count() >= 1.0)
        {
            double hz = write_count / elapsed.count();
            RCLCPP_INFO(this->get_logger(), "Motor Control Rate: %.2f Hz", hz);
            last_print_time = current_time;
            write_count = 0;
        }
    }

    switch(current_state)
    {
        case ControlState::WRITE_PACKET:
            handle_write_packet();
            break;
        case ControlState::READ_PACKET:
            handle_read_packet();
            break;
    }
}

void MainControlNode::transition_to(ControlState new_state)
{
    current_state = new_state;
}

// 모터 상태값 받기
void MainControlNode::handle_read_packet()
{
    static int fail_count = 0;
    static int success_count = 0;

    std::vector<bool> current_cycle_updated(all_motors_.size(), false);
    uint32_t rx_id;
    std::vector<uint8_t> rx_data;

    // 각 CAN 인터페이스에서 non-blocking 수신
    for (auto& group : can_groups_)
    {
        while (group.transport->receive(rx_id, rx_data, 0))
        {
            uint8_t motor_id = RobStrideProtocol::getMotorIdFromCanId(rx_id);

            for (auto& motor : group.motors)
            {
                if (motor->getMotorId() == motor_id)
                {
                    if (motor->processPacket(rx_id, rx_data))
                    {
                        // all_motors_에서의 인덱스 찾기
                        for (size_t idx = 0; idx < all_motors_.size(); idx++)
                        {
                            if (all_motors_[idx] == motor)
                            {
                                current_cycle_updated[idx] = true;
                                break;
                            }
                        }
                    }
                    break;
                }
            }
        }
    }
    // static auto last_msg = roa_interfaces::msg::MotorStateArray();
    // last_msg.states.resize(static_cast<int>(all_motors_.size()));
    auto msg = roa_interfaces::msg::MotorStateArray();
    static int seq = 0;
    msg.states.resize(static_cast<int>(all_motors_.size()));
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "motor_states";
    
    
    int data_idx = 0;

    for (size_t i = 0; i < all_motors_.size(); i++)
    {
        if (current_cycle_updated[i])
        {
            success_count++;
            std::lock_guard<std::mutex> lock(command_mutex_);
            float desired_pos = (i < packet_commands_.commands.size()) ? packet_commands_.commands[i].position : 0.0f;
            float wrapped_pos = wrapToPi(static_cast<float>(all_motors_[i]->getPosition()));
            RCLCPP_INFO(this->get_logger(), "[Red Packet] Motor[%zu] Pos: %.3f, Vel: %.3f, Current: %.3f A, Desired Pos: %.3f",
            i, wrapped_pos, (float)all_motors_[i]->getVelocity(), (float)all_motors_[i]->getCurrent(), desired_pos);


            msg.states[i].motor_id = all_motors_[i]->getMotorId();
            // Col 0: position
            msg.states[i].position = wrapped_pos;
           
            // Col 1: velocity
            msg.states[i].velocity = static_cast<float>(all_motors_[i]->getVelocity());

            // Col 2: current
            msg.states[i].current = static_cast<float>(all_motors_[i]->getCurrent());

        }
        else
        {
            fail_count++;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[Read Packet] Motor[%zu] update failed! (Success: %d, Fail: %d)",
                i, success_count, fail_count);
        }
    }
    state_pub->publish(msg);
    // for(size_t i = 0; i < all_motors_.size(); i++)
    // {
    //     double dt = (rclcpp::Time(msg.header.stamp) - rclcpp::Time(last_msg.header.stamp)).seconds();
    //     if (dt > 1e-6)
    //     {
    //         msg.states[i].velocity = (msg.states[i].position - last_msg.states[i].position) / dt;
    //     }
    // }   
    // state_pub_1->publish(msg);
    // last_msg = msg;
    transition_to(ControlState::WRITE_PACKET);
}

void MainControlNode::handle_write_packet()
{
    std::lock_guard<std::mutex> lock(command_mutex_);

    if (packet_initialized_)
    {
        if (!walk_initialized_)
        {
            init_tick_count_++;
            float alpha = static_cast<float>(init_tick_count_) / static_cast<float>(INIT_TOTAL_TICKS);
            if (alpha > 1.0f) alpha = 1.0f;

            for (size_t i = 0; i < all_motors_.size(); i++)
            {
                float start_raw = start_positions_[i];
                float target_pos = wrapToPi(packet_commands_.commands[i].position);

                float diff = target_pos - wrapToPi(start_raw);
                if (diff > static_cast<float>(M_PI))  diff -= 2.0f * static_cast<float>(M_PI);
                if (diff < -static_cast<float>(M_PI)) diff += 2.0f * static_cast<float>(M_PI);

                float interpolated_target = start_raw + alpha * diff;

                all_motors_[i]->sendMotionCommand(
                    packet_commands_.commands[i].torque,
                    interpolated_target,
                    packet_commands_.commands[i].velocity,
                    packet_commands_.commands[i].kp,
                    packet_commands_.commands[i].kd);

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                    "[Init] packet_index=%zu motor_id=%u start=%.3f target=%.3f cmd=%.3f",
                    i,
                    packet_commands_.commands[i].motor_id,
                    start_raw,
                    target_pos,
                    interpolated_target);
            }

            if (init_tick_count_ >= INIT_TOTAL_TICKS)
            {
                walk_initialized_ = true;
                RCLCPP_INFO(this->get_logger(), "[Init] Walk initialized after %d ticks", init_tick_count_);
            }
        }
        else
        {
            for (size_t i = 0; i < all_motors_.size(); i++)
            {
                float raw_pos = static_cast<float>(all_motors_[i]->getPosition());
                float target_pos = wrapToPi(packet_commands_.commands[i].position);

                float diff = target_pos - wrapToPi(raw_pos);
                if (diff > static_cast<float>(M_PI))  diff -= 2.0f * static_cast<float>(M_PI);
                if (diff < -static_cast<float>(M_PI)) diff += 2.0f * static_cast<float>(M_PI);

                float normalized_target = raw_pos + diff;

                all_motors_[i]->sendMotionCommand(
                    packet_commands_.commands[i].torque,
                    normalized_target,
                    packet_commands_.commands[i].velocity,
                    packet_commands_.commands[i].kp,
                    packet_commands_.commands[i].kd);

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "[Write Packet] packet_index=%zu motor_id=%u raw=%.3f target=%.3f diff=%.3f cmd=%.3f",
                    i,
                    packet_commands_.commands[i].motor_id,
                    raw_pos,
                    target_pos,
                    diff,
                    normalized_target);
            }
        }
    }
    else
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[Write Packet] packet_commands_ not initialized yet");
    }

    transition_to(ControlState::READ_PACKET);
}

void MainControlNode::walkCallback(const roa_interfaces::msg::MotorCommandArray::SharedPtr msg)
{
    if (!walk_initialized_)
    {
        static int counter = 0;
        counter++;

        std_msgs::msg::Bool init_msg;
        init_msg.data = false;
        initial_pub->publish(init_msg);

        if (counter > 2) return;
    }
    else
    {
        std_msgs::msg::Bool init_msg;
        init_msg.data = true;
        initial_pub->publish(init_msg);
    }

    std::lock_guard<std::mutex> lock(command_mutex_);

    // header 복사
    packet_commands_.header = msg->header;

    // 들어온 command들을 motor_id 기반으로 packet index에 링크
    for (const auto& cmd : msg->commands)
    {
        auto it = motor_id_to_index_.find(cmd.motor_id);
        if (it == motor_id_to_index_.end())
        {
            RCLCPP_WARN(this->get_logger(),
                "[Walk Callback] Unknown motor_id: %u", cmd.motor_id);
            continue;
        }

        size_t packet_index = it->second;
        packet_commands_.commands[packet_index] = cmd;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[Walk Callback] Link motor_id=%u -> packet_index=%zu | pos=%.3f vel=%.3f kp=%.3f kd=%.3f tq=%.3f",
            cmd.motor_id, packet_index,
            cmd.position, cmd.velocity, cmd.kp, cmd.kd, cmd.torque);
    }

    packet_initialized_ = true;
}

void MainControlNode::torqueCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    bool torque_enable = msg->data;

    for (size_t i = 0; i < all_motors_.size(); i++)
    {
        if (torque_enable)
        {
            if (all_motors_[i]->enable())
            {
                RCLCPP_INFO(this->get_logger(), "[Torque Callback] Motor[%zu] enabled successfully", i);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "[Torque Callback] Failed to enable motor[%zu]", i);
            }
        }
        else
        {
            if (all_motors_[i]->disable())
            {
                RCLCPP_INFO(this->get_logger(), "[Torque Callback] Motor[%zu] disabled successfully", i);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "[Torque Callback] Failed to disable motor[%zu]", i);
            }
        }
    }
}

// 메인 함수
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MainControlNode>();

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(node->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();

    return 0;
}
