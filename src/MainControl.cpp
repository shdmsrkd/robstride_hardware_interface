#include "robstride_rdk_ros2/MainControl.hpp"
<<<<<<< HEAD
#include <thread>
=======
>>>>>>> e02adc99855708c0aeab7727ed6ab512e364c563

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

MainControlNode::MainControlNode(const rclcpp::NodeOptions & options)
<<<<<<< HEAD
    : LifecycleNode("main_control_node", options), current_state(ControlState::WRITE_PACKET)
=======
    : LifecycleNode("main_control_node", options), current_state(ControlState::READ_PACKET)
>>>>>>> e02adc99855708c0aeab7727ed6ab512e364c563
{
    RCLCPP_INFO(this->get_logger(), "MainControlNode created");
    walk_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "walk2interface", 10,
        std::bind(&MainControlNode::walkCallback, this, std::placeholders::_1)
    );
}

<<<<<<< HEAD
=======
void MainControlNode::initParameters()
{
    declare_parameter("port_name", "/dev/ttyUSB0");
    declare_parameter("baud_rate", 1000000);
    declare_parameter("motor_type", std::vector<int64_t>{06});
    declare_parameter("motor_ids", std::vector<int64_t>{1});
}

>>>>>>> e02adc99855708c0aeab7727ed6ab512e364c563
MainControlNode::~MainControlNode()
{
    RCLCPP_INFO(this->get_logger(), "MainControlNode destroyed");
}

<<<<<<< HEAD
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

CallbackReturn MainControlNode::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "[Configure] Configuring...");
=======
CallbackReturn MainControlNode::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "Configuring...");
>>>>>>> e02adc99855708c0aeab7727ed6ab512e364c563

    // 파라미터 초기화
    initParameters();

<<<<<<< HEAD
    auto can_interfaces = get_parameter("can_interfaces").as_string_array();

    // 각 CAN 인터페이스별 초기화
    can_groups_.clear();
    all_motors_.clear();

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

    RCLCPP_INFO(this->get_logger(), "[Configure] Configured successfully with %zu CAN interfaces, %zu total motors",
        can_groups_.size(), all_motors_.size());
=======
    // CAN 인터페이스 초기화
    transport_ = std::make_shared<CanTransport>();
    if (!transport_->open("can0"))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open CAN interface 'can0'");
        return CallbackReturn::FAILURE;
    }

    // YAML 파라미터 저장
    std::vector<int64_t> motor_types = get_parameter("motor_type").as_integer_array();
    std::vector<int64_t> motor_ids = get_parameter("motor_ids").as_integer_array();
    if (motor_types.size() != motor_ids.size())
    {
        RCLCPP_ERROR(this->get_logger(), "motor_type and motor_ids size mismatch");
        return CallbackReturn::FAILURE;
    }

    // 여러 모터 객체 초기화
    motors_.clear();
    for (size_t i = 0; i < motor_ids.size(); i++)
    {
        ActuatorType type = static_cast<ActuatorType>(motor_types[i]);
        uint8_t id = static_cast<uint8_t>(motor_ids[i]);

        auto motor = std::make_shared<RobStrideMotor>(transport_, id, type);
        motors_.push_back(motor);

        RCLCPP_INFO(this->get_logger(), "Motor[%zu] initialized - ID: %d, Type: %d", i, id, motor_types[i]);
    }

    RCLCPP_INFO(this->get_logger(), "Configured successfully with %zu motors", motors_.size());
>>>>>>> e02adc99855708c0aeab7727ed6ab512e364c563
    return CallbackReturn::SUCCESS;
}

CallbackReturn MainControlNode::on_activate(const rclcpp_lifecycle::State &)
{
<<<<<<< HEAD
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
=======
    RCLCPP_INFO(this->get_logger(), "Activating...");

    // 모든 모터 활성화
    for (size_t i = 0; i < motors_.size(); i++)
    {
        if (motors_[i]->enable())
        {
            RCLCPP_INFO(this->get_logger(), "Motor[%zu] enabled successfully", i);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to enable motor[%zu]", i);
>>>>>>> e02adc99855708c0aeab7727ed6ab512e364c563
            return CallbackReturn::FAILURE;
        }
    }

    // 제어 루프 타이머 시작 (100Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&MainControlNode::control_loop, this));

<<<<<<< HEAD
    RCLCPP_INFO(this->get_logger(), "[Activate] Activated successfully with %zu motors", all_motors_.size());
=======
    RCLCPP_INFO(this->get_logger(), "Activated successfully with %zu motors", motors_.size());
>>>>>>> e02adc99855708c0aeab7727ed6ab512e364c563
    return CallbackReturn::SUCCESS;
}

void MainControlNode::control_loop()
{
    // Hz 측정
    static auto last_print_time = std::chrono::steady_clock::now();
    static int loop_count = 0;

    loop_count++;
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = current_time - last_print_time;

    if (elapsed.count() >= 1.0)
    {
        double hz = loop_count / elapsed.count();
        RCLCPP_INFO(this->get_logger(), "Control Loop Rate: %.2f Hz", hz);
        last_print_time = current_time;
        loop_count = 0;
    }

<<<<<<< HEAD
    switch(current_state)
    {
        case ControlState::WRITE_PACKET:
            handle_write_packet();
            break;
        case ControlState::READ_PACKET:
            handle_read_packet();
=======
    // 매 주기마다 READ → WRITE 2상으로 진행 (PUB 추가할수도 있음)
    switch(current_state)
    {
        case ControlState::READ_PACKET:  // 패킷 읽기
            handle_read_packet();
            break;
        case ControlState::WRITE_PACKET: // 패킷 쓰기
            handle_write_packet();
            break;
        case ControlState::ERROR:        // 오류 처리
            handle_error();
            break;
        default:
>>>>>>> e02adc99855708c0aeab7727ed6ab512e364c563
            break;
    }
}

// 모터 상태값 받기
void MainControlNode::handle_read_packet()
{
    static int fail_count = 0;
    static int success_count = 0;

<<<<<<< HEAD
    std::vector<bool> current_cycle_updated(all_motors_.size(), false);
    uint32_t rx_id;
    std::vector<uint8_t> rx_data;

    // 각 CAN 인터페이스에서 수신 (타임아웃으로 응답 대기)
    for (auto& group : can_groups_)
    {
        // 각 CAN 버스에서 최대 모터 수만큼 응답을 받으려 시도
        int max_reads = static_cast<int>(group.motors.size()) * 3; // 여유분 포함
        for (int r = 0; r < max_reads; r++)
        {
            if (!group.transport->receive(rx_id, rx_data, 2))  // 2ms 타임아웃
                break;
            uint8_t motor_id = RobStrideProtocol::getMotorIdFromCanId(rx_id);
            uint8_t type = RobStrideProtocol::getTypeFromCanId(rx_id);
            RCLCPP_DEBUG(this->get_logger(), "[Read] %s RX CAN_ID=0x%08X, motor_id=%d, type=0x%02X, data_len=%zu",
                group.interface_name.c_str(), rx_id, motor_id, type, rx_data.size());

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
=======
    std::vector<bool> current_cycle_updated(motors_.size(), false);
    uint32_t rx_id;
    std::vector<uint8_t> rx_data;

    // non-blocking (데이터 없으면 즉시 리턴)
    while (transport_->receive(rx_id, rx_data, 0))
    {
        uint8_t motor_id = RobStrideProtocol::getMotorIdFromCanId(rx_id);

        for (size_t i = 0; i < motors_.size(); i++)
        {
            if (motors_[i]->getMotorId() == motor_id)
            {
                if (motors_[i]->processPacket(rx_id, rx_data))
                {
                    current_cycle_updated[i] = true;
                }
                break;
>>>>>>> e02adc99855708c0aeab7727ed6ab512e364c563
            }
        }
    }

<<<<<<< HEAD
    for (size_t i = 0; i < all_motors_.size(); i++)
=======
    for (size_t i = 0; i < motors_.size(); i++)
>>>>>>> e02adc99855708c0aeab7727ed6ab512e364c563
    {
        if (current_cycle_updated[i])
        {
            success_count++;
            std::lock_guard<std::mutex> lock(command_mutex_);
            float desired_pos = (i < motor_commands_.size()) ? motor_commands_[i].position : 0.0f;
<<<<<<< HEAD
            RCLCPP_INFO(this->get_logger(), "[Read Packet] Motor[%zu] Pos: %.3f, Vel: %.3f, Current: %.3f A, Desired Pos: %.3f",
            i, (float)all_motors_[i]->getPosition(), (float)all_motors_[i]->getVelocity(), (float)all_motors_[i]->getCurrent(), desired_pos);
=======
            RCLCPP_INFO(this->get_logger(),
            "Motor[%zu] Pos: %.3f, Vel: %.3f, Current: %.3f A, Desired Pos: %.3f",
            i, (float)motors_[i]->getPosition(),
            (float)motors_[i]->getVelocity(),
            (float)motors_[i]->getCurrent(),
            desired_pos);
>>>>>>> e02adc99855708c0aeab7727ed6ab512e364c563
        }
        else
        {
            fail_count++;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
<<<<<<< HEAD
                "[Read Packet] Motor[%zu] update failed! (Success: %d, Fail: %d)",
=======
                "Motor[%zu] update failed! (Success: %d, Fail: %d)",
>>>>>>> e02adc99855708c0aeab7727ed6ab512e364c563
                i, success_count, fail_count);
        }
    }

    transition_to(ControlState::WRITE_PACKET);
}

<<<<<<< HEAD
=======
void MainControlNode::publish_status()
{
    // 상태 메시지 생성
    auto status_msg = std_msgs::msg::Float32MultiArray();
    status_msg.layout.dim.resize(2);
    status_msg.layout.dim[0].label = "motors";
    status_msg.layout.dim[0].size = motors_.size();
    status_msg.layout.dim[0].stride = motors_.size() * 6; // 6개의 값 (position, velocity, torque, temperature, current, desired_position)
    status_msg.layout.dim[1].label = "values";
    status_msg.layout.dim[1].size = 6;
    status_msg.layout.dim[1].stride = 6;

    // 데이터 채우기
    std::lock_guard<std::mutex> lock(command_mutex_);
    for (size_t i = 0; i < motors_.size(); i++)
    {
        status_msg.data.push_back(motors_[i]->getPosition());
        status_msg.data.push_back(motors_[i]->getVelocity());
        status_msg.data.push_back(motors_[i]->getTorque());
        status_msg.data.push_back(motors_[i]->getTemperature());
        status_msg.data.push_back(motors_[i]->getCurrent());
        float desired_pos = (i < motor_commands_.size()) ? motor_commands_[i].position : 0.0f;
        status_msg.data.push_back(desired_pos);
    }

    // 메시지 퍼블리시
    status_pub->publish(status_msg);
}

>>>>>>> e02adc99855708c0aeab7727ed6ab512e364c563
void MainControlNode::handle_write_packet()
{
    std::lock_guard<std::mutex> lock(command_mutex_);

    // 명령이 있으면 전송, 없으면 이전 명령 유지
    if(!motor_commands_.empty())
    {
<<<<<<< HEAD
        for(size_t i = 0; i < all_motors_.size() && i < motor_commands_.size(); i++)
        {
            all_motors_[i]->sendMotionCommand(
                motor_commands_[i].torque, (float)motor_commands_[i].position,
                (float)motor_commands_[i].velocity, (float)motor_commands_[i].kp, (float)motor_commands_[i].kd);
=======
        for(size_t i = 0; i < motors_.size() && i < motor_commands_.size(); i++)
        {
            motors_[i]->sendMotionCommand(
                motor_commands_[i].torque,
                (float)motor_commands_[i].position,
                (float)motor_commands_[i].velocity,
                motor_commands_[i].kp,
                motor_commands_[i].kd);
>>>>>>> e02adc99855708c0aeab7727ed6ab512e364c563
        }
    }
    else
    {
<<<<<<< HEAD
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[Write Packet] motor_commands_ is empty!");
=======
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "motor_commands_ is empty!");
>>>>>>> e02adc99855708c0aeab7727ed6ab512e364c563
    }

    transition_to(ControlState::READ_PACKET);
}

<<<<<<< HEAD
=======
void MainControlNode::handle_error()
{
    // 오류 처리 작업 수행
}

>>>>>>> e02adc99855708c0aeab7727ed6ab512e364c563
void MainControlNode::transition_to(ControlState new_state)
{
    current_state = new_state;
}

void MainControlNode::walkCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    // 데이터 유효성 검사
    if (msg->layout.dim.size() < 2)
    {
<<<<<<< HEAD
        RCLCPP_WARN(this->get_logger(), "[Walk Callback] This is not 2D matrix data");
=======
        RCLCPP_WARN(this->get_logger(), "2차원 행렬 데이터가 아닙니다.");
>>>>>>> e02adc99855708c0aeab7727ed6ab512e364c563
        return;
    }

    // 행렬 설정: 행 = 모터 개수, 열 = 5 (position, velocity, torque, kp, kd)
    int height_rows = msg->layout.dim[0].size;  // 모터 개수
    int width_cols = msg->layout.dim[1].size;   // 5개 값

    if (width_cols != 5)
    {
<<<<<<< HEAD
        RCLCPP_WARN(this->get_logger(), "[Walk Callback] Column size is not 5 : %d", width_cols);
        return;
    }

    if (height_rows != static_cast<int>(all_motors_.size()))
    {
        RCLCPP_WARN(this->get_logger(), "[Walk Callback] Row size does not match the number of motors : %d (Expected: %zu)", height_rows, all_motors_.size());
=======
        RCLCPP_WARN(this->get_logger(), "열 크기가 5가 아닙니다: %d", width_cols);
        return;
    }

    if (height_rows != motors_.size())
    {
        RCLCPP_WARN(this->get_logger(), "행 크기가 모터 개수와 다릅니다: %d (예상: %zu)", height_rows, motors_.size());
>>>>>>> e02adc99855708c0aeab7727ed6ab512e364c563
        return;
    }

    std::vector<MotorCommand> temp_commands;
    temp_commands.reserve(height_rows);

    for (int r = 0; r < height_rows; r++)
    {
        int idx = r * width_cols;

        MotorCommand cmd;
        cmd.torque   = msg->data[idx + 0];
        cmd.position = msg->data[idx + 1];
        cmd.velocity = msg->data[idx + 2];
        cmd.kp       = msg->data[idx + 3];
        cmd.kd       = msg->data[idx + 4];

        temp_commands.push_back(cmd);
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    motor_commands_ = std::move(temp_commands);
<<<<<<< HEAD
=======
    receive_flag = true;

    // RCLCPP_INFO(this->get_logger(), "Received commands for %zu motors", motor_commands_.size());
>>>>>>> e02adc99855708c0aeab7727ed6ab512e364c563
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
