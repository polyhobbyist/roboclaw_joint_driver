#include <boost/asio.hpp>
#include <future>

#include "roboclaw.h"

void Roboclaw::write(bool crc, std::vector<uint8_t> data)
{
    if (crc)
    {
        uint16_t crc = 0;
        for (auto byte : data)
        {
            updateCRC(crc, byte);
        }
        data.push_back(crc >> 8);
        data.push_back(crc & 0xFF);
    }

    serial_->write_some(boost::asio::buffer(data));
}

#include <boost/asio/serial_port.hpp> // Include the missing header file

uint8_t Roboclaw::read(uint8_t& data)
{
  return serial_->read_some(boost::asio::buffer(&data, 1));
}

void Roboclaw::updateCRC(uint16_t& crc, uint8_t data)
{
    crc = crc ^ ((uint16_t)data << 8);
    for (int i = 0; i < 8; i++)
    {
        if (crc & 0x8000)
        {
            crc = (crc << 1) ^ 0x1021;
        }
        else
        {
            crc <<= 1;
        }
    }
}

void Roboclaw::updateCRC(uint16_t& crc, std::vector<uint8_t>& data)
{
    for (uint8_t byte : data)
    {
        updateCRC(crc, byte);
    }
}


bool Roboclaw::open(const std::string& port, uint32_t baud_rate, int roboclawAddress)
{
  port_ = port;
  baud_rate_ = baud_rate;
  roboclawAddress_ = roboclawAddress;
  ios_ = std::make_shared<boost::asio::io_service>();
  ios_thread_ = std::thread([this]() { ios_->run(); });


  serial_ = std::make_shared<boost::asio::serial_port>(*ios_);
  serial_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
  try 
  {
    serial_->open(port_);
    return true;
  }
  catch (boost::system::system_error& e)
  {
    return false;
  }
}

void Roboclaw::close()
{
  serial_->close();
  ios_->stop();
  ios_thread_.join();
}

std::string Roboclaw::getVersion()
{
  uint16_t crc = 0;
  std::vector<uint8_t> data = { roboclawAddress_, Roboclaw::Command::GetVersion };
  updateCRC(crc, data);
  write(false, data);

  std::stringstream ss;
  uint8_t readData;
  while (read(readData))
  {
    updateCRC(crc, readData);
    if (readData == 0)
    {
      uint16_t readCRC = 0;
      read(readData);
      updateCRC(readCRC, readData);
      read(readData);
      updateCRC(readCRC, readData);
      if (crc == readCRC)
      {
        return ss.str();
      }
      else
      {
        throw std::runtime_error("CRC mismatch");
      }

      break;
    }

    ss << readData;
  }

  return ss.str();
}
