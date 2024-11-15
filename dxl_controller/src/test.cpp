#include <cstdio>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/float64.hpp>

class SerialTest : public rclcpp::Node
{
public:
  SerialTest() : Node("serial_test")
  {
    try
    {
      serial_.setPort("/dev/ttyUSB0"); // 시리얼 포트 설정
      serial_.setBaudrate(1000000); // Dynamixel 설정 보드레이트
      serial_.open();
    }
    catch (const serial::IOException &e) // 예외 처리
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
    pos_sub_ = this->create_subscription<std_msgs::msg::Float64>("pos_run", 10, std::bind(&SerialTest::pos_callback, this, std::placeholders::_1));
    vel_sub_ = this->create_subscription<std_msgs::msg::Float64>("vel_run", 10, std::bind(&SerialTest::vel_callback, this, std::placeholders::_1));
    acc_sub_ = this->create_subscription<std_msgs::msg::Float64>("acc_run", 10, std::bind(&SerialTest::acc_callback, this, std::placeholders::_1));

    // Torque Enable 및 모드 설정
    enableTourque();
  }

private:
  serial::Serial serial_;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float64>> pos_sub_;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float64>> vel_sub_;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float64>> acc_sub_;

  // 위치 제어
  void pos_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    uint32_t goal_position = static_cast<uint32_t>(msg->data);
    setPosition(goal_position);  // 주소 116: Goal Position
  }

  // 속도 제어
  void vel_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    uint16_t goal_velocity = static_cast<uint16_t>(msg->data);
    setVelocity(goal_velocity);  // 주소 112: Goal Velocity
  }

  // 가속도 제어
  void acc_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    uint16_t goal_acceleration = static_cast<uint16_t>(msg->data);
    setAcceleration(goal_acceleration);  // 주소 108: Goal Acceleration
  }

  void enableTourque()
  {
    // Torque Enable 설정
    uint8_t torque_buffer[14] = {0xFF, 0xFF, 0xFD, 0x00, 0x00, 0x06, 0x00, 0x03, 0x40, 0x00, 0x01};
    uint16_t torque_crc = update_crc(0, torque_buffer, 11);
    torque_buffer[11] = torque_crc & 0xFF;
    torque_buffer[12] = (torque_crc >> 8) & 0xFF;
    serial_.write(torque_buffer, 13);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  void setPosition(uint32_t position)
  {
    uint8_t position_buffer[16] = {0xFF, 0xFF, 0xFD, 0x00, 0x00, 0x09, 0x00, 0x03, 0x74, 0x00};
    position_buffer[10] = position & 0xFF;
    position_buffer[11] = (position >> 8) & 0xFF;
    position_buffer[12] = (position >> 16) & 0xFF;
    position_buffer[13] = (position >> 24) & 0xFF;
    uint16_t pos_crc = update_crc(0, position_buffer, 14);
    position_buffer[14] = pos_crc & 0xFF;
    position_buffer[15] = (pos_crc >> 8) & 0xFF;
    serial_.write(position_buffer, 16);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  void setVelocity(uint16_t velocity)
  {
    uint8_t velocity_buffer[12] = {0xFF, 0xFF, 0xFD, 0x00, 0x00, 0x07, 0x00, 0x03, 0x70, 0x00};
    velocity_buffer[10] = velocity & 0xFF;
    velocity_buffer[11] = (velocity >> 8) & 0xFF;
    uint16_t vel_crc = update_crc(0, velocity_buffer, 10);
    velocity_buffer[10] = vel_crc & 0xFF;
    velocity_buffer[11] = (vel_crc >> 8) & 0xFF;
    serial_.write(velocity_buffer, 12);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  void setAcceleration(uint16_t acceleration)
  {
    uint8_t acceleration_buffer[12] = {0xFF, 0xFF, 0xFD, 0x00, 0x00, 0x07, 0x00, 0x03, 0x6C, 0x00};
    acceleration_buffer[10] = acceleration & 0xFF;
    acceleration_buffer[11] = (acceleration >> 8) & 0xFF;
    uint16_t acc_crc = update_crc(0, acceleration_buffer, 10);
    acceleration_buffer[10] = acc_crc & 0xFF;
    acceleration_buffer[11] = (acc_crc >> 8) & 0xFF;
    serial_.write(acceleration_buffer, 12);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }


  // ref: https://emanual.robotis.com/docs/kr/dxl/crc/
  uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
  {
    uint16_t i, j;
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
      0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202};

    for (j = 0; j < data_blk_size; j++)
    {
      i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
      crc_accum = (crc_accum << 8) ^ crc_table[i];
    }
    return crc_accum;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialTest>());
  rclcpp::shutdown();
  return 0;
}
