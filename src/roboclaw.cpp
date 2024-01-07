#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <future>

#include "roboclaw.h"

#define Int32ToBytes(val) (uint8_t)((val) >> 24), (uint8_t)((val) >> 16), (uint8_t)((val) >> 8), (uint8_t)(val)

Roboclaw::Roboclaw()
: port_(""), baud_rate_(0), roboclawAddress_(0),
  motor1_p_(0), motor1_d_(0), motor1_i_(0), motor1_qpps_(0),
  motor2_p_(0), motor2_d_(0), motor2_i_(0), motor2_qpps_(0)
{
}

size_t Roboclaw::write(bool crc, const std::initializer_list<uint8_t>& data)
{
  std::vector<uint8_t> writeData(data);
    if (crc)
    {
        uint16_t crc = 0;
        for (auto byte : data)
        {
            updateCRC(crc, byte);
        }
        writeData.push_back(crc >> 8);
        writeData.push_back(crc & 0xFF);
    }

    try
    {
      size_t written = serial_->write_some(boost::asio::buffer(writeData));
      if (written == writeData.size())
      {
        return written;
      }
      else
      {
        reset();
      }
    }
    catch (boost::system::system_error& e)
    {
      reset();
    }
    return 0;
}

void Roboclaw::reset()
{
  serial_->close();
  serial_->open(port_);
}

uint8_t Roboclaw::read(uint8_t& data)
{
  return serial_->read_some(boost::asio::buffer(&data, 1));
}

void Roboclaw::updateCRC(uint16_t& crc, uint8_t data) const
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

void Roboclaw::updateCRC(uint16_t& crc, const std::initializer_list<uint8_t>& data) const
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
  std::initializer_list<uint8_t> data = { roboclawAddress_, Roboclaw::Command::GetVersion };
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


void Roboclaw::setPID(int motor, float p, float i, float d, uint32_t qpps)
{
  uint32_t ip = (uint32_t)(p * 65536.0);
  uint32_t ii = (uint32_t)(i * 65536.0);
  uint32_t id = (uint32_t)(d * 65536.0);

  write(true, 
    { 
      roboclawAddress_, 
      motor == 0 ? Roboclaw::Command::SetM1Pid : Roboclaw::Command::SetM2Pid, 
      Int32ToBytes(ip), 
      Int32ToBytes(ii), 
      Int32ToBytes(id), 
      Int32ToBytes(qpps)
    }
  );
}

void Roboclaw::setSpeed(int motor, int speed)
{
  write(true, 
    { 
      roboclawAddress_, 
      motor == 0 ? Roboclaw::Command::SetM1Speed : Roboclaw::Command::SetM2Speed, 
      Int32ToBytes(speed)
    }
  );
}

int Roboclaw::getSpeed(int motor)
{
  uint16_t crc = 0;
  std::initializer_list<uint8_t> data = { roboclawAddress_, motor == 0 ? Roboclaw::Command::GetM1Speed : Roboclaw::Command::GetM2Speed };
  updateCRC(crc, data);
  write(false, data);

  uint8_t readData;
  int speed = 0;

  // Read high byte
  read(readData);
  updateCRC(crc, readData);
  speed = readData << 24;

  // Read middle high byte
  read(readData);
  updateCRC(crc, readData);
  speed |= readData << 16;

  // Read middle low byte
  read(readData);
  updateCRC(crc, readData);
  speed |= readData << 8;

  // Read low byte
  read(readData);
  updateCRC(crc, readData);
  speed |= readData;

  // Read Direction
  read(readData);
  updateCRC(crc, readData);
  if (readData != 0)
  {
    speed = -speed;
  }

  // Read Response CRC
  uint16_t readCRC = 0;
  read(readData);
  readCRC = readData << 8;
  read(readData);
  readCRC |= readData;

  if (crc != readCRC)
  {
    throw std::runtime_error("CRC mismatch");
  }

  return speed;
}


int Roboclaw::getEncoder(int motor)
{
  uint16_t crc = 0;
  std::initializer_list<uint8_t> data = { roboclawAddress_, motor == 0 ? Roboclaw::Command::GetM1Encoder : Roboclaw::Command::GetM2Encoder };
  updateCRC(crc, data);
  write(false, data);

  uint8_t readData;
  int encoder = 0;

  // Read high byte
  read(readData);
  updateCRC(crc, readData); 
  encoder = readData << 24;

  // Read middle high byte
  read(readData);
  updateCRC(crc, readData);
  encoder |= readData << 16;

  // Read middle low byte
  read(readData);
  updateCRC(crc, readData);
  encoder |= readData << 8;

  // Read low byte
  read(readData);
  updateCRC(crc, readData);
  encoder |= readData;

  // Read Response CRC
  uint16_t readCRC = 0;
  read(readData);
  readCRC = readData << 8;
  read(readData);
  readCRC |= readData;

  if (crc != readCRC)
  {
    throw std::runtime_error("CRC mismatch");
  }

  return encoder;
}

void Roboclaw::setEncoder(int motor, int encoder)
{
  if (motor > 1) 
  {
    throw std::runtime_error("Invalid motor");
  }

  write(true, 
    { 
      roboclawAddress_, 
      motor == 0 ? Roboclaw::Command::SetM1Encoder : Roboclaw::Command::SetM2Encoder, 
      Int32ToBytes(encoder)
    }
  );

}


void Roboclaw::resetEncoders()
{
  setEncoder(0, 0);
  setEncoder(1, 0);
}





