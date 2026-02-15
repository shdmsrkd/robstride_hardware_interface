#include "robstride_rdk_ros2/CanTransport.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <poll.h>

CanTransport::CanTransport() {}

CanTransport::~CanTransport() {
    close();
}

bool CanTransport::open(const std::string& interface_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    interface_name_ = interface_name;

    // 소캣 생성
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0)
    {
        perror("Socket creation failed");
        return false;
    }

    // 인터페이스 이름으로 인덱스 가져오기
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);

    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        perror("Interface index retrieval failed");
        close();
        return false;
    }

    // 소캣 주소 설정 및 바인딩
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("Socket bind failed");
        close();
        return false;
    }

    std::cout << "[CanTransport] Opened " << interface_name << " successfully." << std::endl;
    return true;
}

void CanTransport::close() {
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

bool CanTransport::send(uint32_t can_id, const std::vector<uint8_t>& data) {
    // 소캣이 열려있는지, 데이터 크기가 8바이트 이하인지 확인
    if (socket_fd_ < 0) return false;
    if (data.size() > 8) {
        std::cerr << "Error: Data size > 8 bytes" << std::endl;
        return false;
    }

    // CAN 프레임 구성
    struct can_frame frame;
    frame.can_id = can_id | CAN_EFF_FLAG; // 항상 확장 프레임 사용
    frame.can_dlc = static_cast<uint8_t>(data.size());
    std::memcpy(frame.data, data.data(), data.size());

    std::lock_guard<std::mutex> lock(mutex_);
    int nbytes = write(socket_fd_, &frame, sizeof(frame));

    if (nbytes != sizeof(frame))
    {
        perror("Write failed");
        return false;
    }
    return true;
}

bool CanTransport::receive(uint32_t& can_id, std::vector<uint8_t>& data, int timeout_ms)
{
    if (socket_fd_ < 0) throw std::runtime_error("CAN socket is not open");

    struct pollfd pfd;
    pfd.fd = socket_fd_;
    pfd.events = POLLIN;

    // poll을 사용하여 타임아웃 처리 (CPU 점유율 낮춤)
    int ret = poll(&pfd, 1, timeout_ms);

    if (ret < 0)
    {
        perror("Poll error");
        return false;
    }
    else if (ret == 0)
    {
        return false;
    }

    // revent -> poll()이 채운 실제 발생한 이벤트
    if (pfd.revents & POLLIN)
    {
        struct can_frame frame;
        ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));

        if (nbytes < 0)
        {
            perror("Read error");
            return false;
        }

        // 확장 프레임 여부 확인
        if (!(frame.can_id & CAN_EFF_FLAG))
        {
            throw std::runtime_error("Received standard CAN frame, expected extended frame");
        }

        can_id = frame.can_id & CAN_EFF_MASK;
        data.assign(frame.data, frame.data + frame.can_dlc);
        return true;
    }

    return false;
}
