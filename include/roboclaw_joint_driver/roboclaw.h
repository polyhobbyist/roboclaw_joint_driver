#pragma once

#include <boost/asio.hpp>
#include <future>

// Implements the RoboClaw USB interface using Boost ASIO
class Roboclaw
{
    std::string port_;
    uint32_t baud_rate_;
    boost::asio::serial_port serial_;
    uint8_t roboclawAddress_;

    typedef enum RoboclawCommand : uint8_t {
        GetVersion = 21,
    };

    void write(bool crc, std::vector<uint8_t> data);
    uint8_t read(uint8_t& data);

    void updateCRC(uint16_t& crc, uint8_t data);
    void updateCRC(uint16_t& crc, std::vector<uint8_t>& data);

public:
    Roboclaw(boost::asio::io_service& io_service, const std::string& port, uint32_t baud_rate, int roboclawAddress);
    void open();
    void close();
    std::string getVersion();
};
