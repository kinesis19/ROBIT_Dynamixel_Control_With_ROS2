// #ifndef SERIAL_PROCESS_HPP
// #define SERIAL_PROCESS_HPP

// #include <rclcpp/rclcpp.hpp>
// #include <serial/serial.h>
// #include <std_msgs/msg/float64.hpp>
// #include <vector>
// #include <thread>
// #include <chrono>

// class SerialProcess : public rclcpp::Node
// {
// public:
//     SerialProcess();

// private:
//     // 콜백 메서드
//     void pos_callback(const std_msgs::msg::Float64::SharedPtr msg);
//     void vel_callback(const std_msgs::msg::Float64::SharedPtr msg);
//     void acc_callback(const std_msgs::msg::Float64::SharedPtr msg);

//     // 데이터 전송 메서드
//     void enableTorque();
//     void sendPosition(uint32_t position);
//     void sendVelocity(uint16_t velocity);
//     void sendAcceleration(uint16_t acceleration);
//     void sendToDxl(uint16_t address, uint32_t value, uint8_t size);

//     // CRC 계산 메서드: ref -> dynamixel wiki
//     uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);

    
//     serial::Serial serial_;
//     std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float64>> pos_sub_;
//     std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float64>> vel_sub_;
//     std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float64>> acc_sub_;
// };

// #endif // SERIAL_PROCESS_HPP

#ifndef SERIAL_PROCESS_HPP
#define SERIAL_PROCESS_HPP

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/float64.hpp>
#include <vector>

class SerialProcess : public rclcpp::Node
{
public:
    SerialProcess();

private:
    serial::Serial serial_;

    uint8_t motor_id_;
    uint32_t goal_position_;
    uint16_t goal_velocity_;
    uint16_t goal_acceleration_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr acceleration_sub_;

    // Callback methods
    void positionCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void velocityCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void accelerationCallback(const std_msgs::msg::Float64::SharedPtr msg);

    // Control methods
    void enableTorque();
    void configureOperatingMode();
    void configureDriveMode();

    // Packet handling
    void sendPacket(uint16_t address, uint32_t data, uint8_t data_size);
    uint16_t calculateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
};

#endif // SERIAL_PROCESS_HPP
