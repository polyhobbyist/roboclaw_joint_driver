#pragma once

#include <boost/asio.hpp>
#include <future>

// Implements the RoboClaw USB interface using Boost ASIO
class Roboclaw
{
    std::string port_;
    uint32_t baud_rate_;
    std::shared_ptr<boost::asio::serial_port> serial_;
    uint8_t roboclawAddress_;

    enum Command : uint8_t {
        GetM1Encoder = 16,
        GetM2Encoder = 17,
        GetVersion = 21,
        SetM1Encoder = 22,
        SetM2Encoder = 23,
        SetM1Pid = 28,
        SetM2Pid = 29,
        GetM1Speed = 30,
        GetM2Speed = 31,
        SetM1Speed = 35,
        SetM2Speed = 36,
    };

    size_t write(bool crc, const std::initializer_list<uint8_t>& data);
    uint8_t read(uint8_t& data);

    void updateCRC(uint16_t& crc, uint8_t data) const;
    void updateCRC(uint16_t& crc, const std::initializer_list<uint8_t>& data) const;

    void reset();

public:
    Roboclaw();
    bool open(const std::string& port, uint32_t baud_rate, int roboclawAddress);
    void close();
    std::string getVersion();
    void setSpeed(int motor, int speed);
    int getSpeed(int motor);
    int getEncoder(int motor);
    void setEncoder(int motor, int encoder);
    void resetEncoders();
    void setPID(int motor, float p, float i, float d, uint32_t qpps);


     std::shared_ptr<boost::asio::io_service> ios_;
     std::thread ios_thread_;


     float motor1_p_;
     float motor1_d_;
     float motor1_i_;
     uint32_t motor1_qpps_;

     float motor2_p_;
     float motor2_d_;
     float motor2_i_;
     uint32_t motor2_qpps_;
     

};
