// #include "../include/dxl_controller/serial_process.hpp"

// SerialProcess::SerialProcess() : Node("serial_process")
// {
//     try
//     {
//         serial_.setPort("/dev/ttyUSB0"); // 시리얼 포트 설정
//         serial_.setBaudrate(1000000); // Dynamixel 설정 보드레이트
//         serial_.open(); // 포트 열기
//     }
//     catch (const serial::IOException &e) // 예외 처리
//     {
//         RCLCPP_ERROR(this->get_logger(), "Unable to open port: %s", e.what());
//         rclcpp::shutdown();
//         return;
//     }

//     if (serial_.isOpen())
//     {
//         RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
//     }
//     else
//     {
//         RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
//         rclcpp::shutdown();
//         return;
//     }

//     // 서브스크라이버 설정: 위치(pos), 속도(vel), 가속도(acc) 구독이 제시된 조건임
//     pos_sub_ = this->create_subscription<std_msgs::msg::Float64>("pos", 10, std::bind(&SerialProcess::pos_callback, this, std::placeholders::_1));
//     vel_sub_ = this->create_subscription<std_msgs::msg::Float64>("vel", 10, std::bind(&SerialProcess::vel_callback, this, std::placeholders::_1));
//     acc_sub_ = this->create_subscription<std_msgs::msg::Float64>("acc", 10, std::bind(&SerialProcess::acc_callback, this, std::placeholders::_1));

//     // Torque Enable 설정: dxl 초기화
//     enableTorque();
// }

// // 위치 제어 처리 콜백 메서드
// void SerialProcess::pos_callback(const std_msgs::msg::Float64::SharedPtr msg)
// {
//     // Adr: 116 (G.P.)
//     sendToDxl(116, static_cast<uint32_t>(msg->data), 4);
//     RCLCPP_INFO(this->get_logger(), "Position 업데이트: %f", msg->data);
// }

// // 속도 제어 처리 콜백 메서드
// void SerialProcess::vel_callback(const std_msgs::msg::Float64::SharedPtr msg)
// {
//     // Adr: 112 (G.P.)
//     sendToDxl(112, static_cast<uint16_t>(msg->data), 2);
//     RCLCPP_INFO(this->get_logger(), "Velocity 업데이트: %f", msg->data);
// }

// // 가속도 제어 처리 콜백 메서드
// void SerialProcess::acc_callback(const std_msgs::msg::Float64::SharedPtr msg)
// {
//     // Adr:108 (G.P.)
//     sendToDxl(108, static_cast<uint16_t>(msg->data), 2);
//     RCLCPP_INFO(this->get_logger(), "Acceleration 업데이트: %f", msg->data);
// }

// /* Torque Enable 설정
// * dxl 제어를 위해서 Torque 활성화가 선수행 되어야 한다고 함
// * dxl은 기본적으로 전원을 공급하면 Torque가 비활성화 상태임 -> 안정성을 위해 비활성화 처리 되어 있는 것으로 알고 있음
// * dxl을 제어하기 위해서 Torque를 활성화 해줘야 하는 것임
// */ 

// void SerialProcess::enableTorque()
// {
//     sendToDxl(64, 1, 1); // 주소 64, 값 1
//     RCLCPP_INFO(this->get_logger(), "Torque enabled");
// }

// // // 위치 전송
// // void SerialProcess::sendPosition(uint32_t position)
// // {
// //     uint8_t position_buffer[16] = {0xFF, 0xFF, 0xFD, 0x00, 0x00, 0x09, 0x00, 0x03, 0x74, 0x00};
// //     position_buffer[10] = position & 0xFF; // 위치 데이터 LSB
// //     position_buffer[11] = (position >> 8) & 0xFF; // 위치 데이터 MSB
// //     position_buffer[12] = (position >> 16) & 0xFF;
// //     position_buffer[13] = (position >> 24) & 0xFF;
// //     uint16_t pos_crc = update_crc(0, position_buffer, 14);
// //     position_buffer[14] = pos_crc & 0xFF;
// //     position_buffer[15] = (pos_crc >> 8) & 0xFF;
// //     serial_.write(position_buffer, 16);
// //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
// // }

// // // 속도 전송
// // void SerialProcess::sendVelocity(uint16_t velocity)
// // {
// //     uint8_t velocity_buffer[12] = {0xFF, 0xFF, 0xFD, 0x00, 0x00, 0x07, 0x00, 0x03, 0x70, 0x00};
// //     velocity_buffer[10] = velocity & 0xFF; // 속도 데이터 LSB
// //     velocity_buffer[11] = (velocity >> 8) & 0xFF; // 속도 데이터 MSB
// //     uint16_t vel_crc = update_crc(0, velocity_buffer, 10);
// //     velocity_buffer[10] = vel_crc & 0xFF;
// //     velocity_buffer[11] = (vel_crc >> 8) & 0xFF;
// //     serial_.write(velocity_buffer, 12);
// //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
// // }

// // // 가속도 전송
// // void SerialProcess::sendAcceleration(uint16_t acceleration)
// // {
// //     uint8_t acceleration_buffer[12] = {0xFF, 0xFF, 0xFD, 0x00, 0x00, 0x07, 0x00, 0x03, 0x6C, 0x00};
// //     acceleration_buffer[10] = acceleration & 0xFF; // 가속도 데이터 LSB
// //     acceleration_buffer[11] = (acceleration >> 8) & 0xFF; // 가속도 데이터 MSB
// //     uint16_t acc_crc = update_crc(0, acceleration_buffer, 10);
// //     acceleration_buffer[10] = acc_crc & 0xFF;
// //     acceleration_buffer[11] = (acc_crc >> 8) & 0xFF;
// //     serial_.write(acceleration_buffer, 12);
// //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
// // }

// // CRC 계산
// // ref: https://emanual.robotis.com/docs/kr/dxl/crc/
// uint16_t SerialProcess::update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
// {
//     unsigned short i, j;
//     unsigned short crc_table[256] = {
//         0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
//         0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
//         0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
//         0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
//         0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
//         0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
//         0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
//         0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
//         0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
//         0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
//         0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
//         0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
//         0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
//         0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
//         0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
//         0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
//         0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
//         0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
//         0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
//         0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
//         0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
//         0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
//         0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
//         0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
//         0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
//         0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
//         0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
//         0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
//         0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
//         0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
//         0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
//         0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
//     };

//     for(j = 0; j < data_blk_size; j++)
//     {
//         i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
//         crc_accum = (crc_accum << 8) ^ crc_table[i];
//     }

//     return crc_accum;
// }

// // Dxl로 패킷 보내는 메서드
// void SerialProcess::sendToDxl(uint16_t address, uint32_t value, uint8_t size)
// {
//     // 패킷 초기화 -> vector 형태
//     std::vector<uint8_t> packet = {0xFF, 0xFF, 0xFD, 0x00, 0x00, static_cast<uint8_t>(size + 3), 0x00, 0x03};
//     // Dxl의 메모리 주소 추가
//     packet.push_back(address & 0xFF);
//     packet.push_back((address >> 8) & 0xFF);

//     // 데이터 추가
//     for (uint8_t i = 0; i < size; ++i)
//     {
//         packet.push_back(static_cast<uint8_t>(value >> (8 * i)) & 0xFF);
//     }
    
//     // crc 걔산
//     uint16_t crc = update_crc(0, packet.data(), packet.size());
//     packet.push_back(crc & 0xFF);
//     packet.push_back((crc >> 8) & 0xFF);

//     // 패킷 전송
//     serial_.write(packet);
//     RCLCPP_DEBUG(this->get_logger(), "Packet sent to motor: Address=0x%X, Value=0x%X, Size=%d", address, value, size);
//     std::this_thread::sleep_for(std::chrono::milliseconds(100));
// }

#include "../include/dxl_controller/serial_process.hpp"

SerialProcess::SerialProcess()
    : Node("serial_process"), motor_id_(1), goal_position_(0), goal_velocity_(0), goal_acceleration_(0)
{
    try
    {
        serial_.setPort("/dev/ttyUSB0");
        serial_.setBaudrate(1000000);
        serial_.open();
    }
    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to open port: %s", e.what());
        rclcpp::shutdown();
        return;
    }

    if (serial_.isOpen())
    {
        RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
        rclcpp::shutdown();
        return;
    }

    // Subscription to topics
    position_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "pos", 10, std::bind(&SerialProcess::positionCallback, this, std::placeholders::_1));
    velocity_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "vel", 10, std::bind(&SerialProcess::velocityCallback, this, std::placeholders::_1));
    acceleration_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "acc", 10, std::bind(&SerialProcess::accelerationCallback, this, std::placeholders::_1));

    configureDriveMode();
    configureOperatingMode();
    enableTorque();
}

// Callback methods
void SerialProcess::positionCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    goal_position_ = static_cast<uint32_t>(msg->data);
    RCLCPP_INFO(this->get_logger(), "Position set to: %u", goal_position_);
    sendPacket(116, goal_position_, 4); // Address 116: Goal Position
}

void SerialProcess::velocityCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    goal_velocity_ = static_cast<uint16_t>(msg->data);
    RCLCPP_INFO(this->get_logger(), "Velocity set to: %u", goal_velocity_);
    sendPacket(112, goal_velocity_, 2); // Address 112: Goal Velocity
}

void SerialProcess::accelerationCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    goal_acceleration_ = static_cast<uint16_t>(msg->data);
    RCLCPP_INFO(this->get_logger(), "Acceleration set to: %u", goal_acceleration_);
    sendPacket(108, goal_acceleration_, 2); // Address 108: Goal Acceleration
}

// Control methods
void SerialProcess::enableTorque()
{
    sendPacket(64, 1, 1); // Address 64: Torque Enable
    RCLCPP_INFO(this->get_logger(), "Torque enabled");
}

void SerialProcess::configureDriveMode()
{
    sendPacket(10, 0, 1); // Address 10: Drive Mode
    RCLCPP_INFO(this->get_logger(), "Drive mode configured");
}

void SerialProcess::configureOperatingMode()
{
    sendPacket(11, 4, 1); // Address 11: Operating Mode (Extended Position Control)
    RCLCPP_INFO(this->get_logger(), "Operating mode configured");
}

// Packet handling
void SerialProcess::sendPacket(uint16_t address, uint32_t data, uint8_t data_size)
{
    std::vector<uint8_t> packet = {0xFF, 0xFF, 0xFD, 0x00, motor_id_, static_cast<uint8_t>(data_size + 3), 0x00, 0x03};
    packet.push_back(address & 0xFF);
    packet.push_back((address >> 8) & 0xFF);

    for (uint8_t i = 0; i < data_size; ++i)
    {
        packet.push_back(static_cast<uint8_t>(data >> (8 * i)) & 0xFF);
    }

    uint16_t crc = calculateCRC(0, packet.data(), packet.size());
    packet.push_back(crc & 0xFF);
    packet.push_back((crc >> 8) & 0xFF);

    serial_.write(packet);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

// CRC calculation
uint16_t SerialProcess::calculateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
    static const uint16_t crc_table[256] = {
      0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
      0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
      0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
      0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
      0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
      0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
      0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
      0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
      0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
      0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
      0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
      0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
      0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
      0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
      0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
      0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
      0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
      0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
      0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
      0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
      0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
      0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
      0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
      0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
      0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
      0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
      0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
      0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
      0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
      0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
      0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
      0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for (uint16_t i = 0; i < data_blk_size; i++)
    {
        uint16_t idx = (crc_accum >> 8) ^ data_blk_ptr[i];
        crc_accum = (crc_accum << 8) ^ crc_table[idx];
    }
    return crc_accum;
}
