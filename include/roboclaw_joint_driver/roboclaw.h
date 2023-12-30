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
        GetVersion = 21,
    };

    void write(bool crc, std::vector<uint8_t> data);
    uint8_t read(uint8_t& data);

    void updateCRC(uint16_t& crc, uint8_t data);
    void updateCRC(uint16_t& crc, std::vector<uint8_t>& data);

public:
    Roboclaw();
    bool open(const std::string& port, uint32_t baud_rate, int roboclawAddress);
    void close();
    std::string getVersion();

     std::shared_ptr<boost::asio::io_service> ios_;
     std::thread ios_thread_;
};
