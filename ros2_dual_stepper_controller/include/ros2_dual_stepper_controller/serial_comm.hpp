
#ifndef SERIAL_COMM_HPP
#define SERIAL_COMM_HPP

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include <string>
#include <vector>

class SerialComm {

  public:
    SerialComm();

    void setSerialPort(const std::string & serial_port);
    void setBaudRate(int baud_rate);

   /* Initialize the serial communication by opening the serial port and configuring it. It is necessary to first set
    * the attributes serial_port_ and baud_rate_ before calling this method.
    */
    hardware_interface::CallbackReturn init();

    bool receive(uint8_t &cmd, std::vector<uint8_t> &data);
    bool send(uint8_t cmd, const std::vector<uint8_t> &data);

    // Get the logger of the SystemInterface.
    rclcpp::Logger get_logger() const { return *logger_; }

    // Variables relative to the communication protocol
    inline static constexpr uint8_t HEADER        = 0xAA;
    inline static constexpr uint8_t VEL_CMD       = 0x01;
    inline static constexpr uint8_t ENC_CMD       = 0x02;
    inline static constexpr uint8_t VEL_DATA_LEN  = 8;
    inline static constexpr uint8_t ENC_DATA_LEN  = 8;
    inline static constexpr size_t  MAX_PACKET_SIZE = 16;

  private:

    int serial_fd_;
    int baud_rate_;
    std::string serial_port_;

    enum State {
      WAIT_HEADER,  // Looking for the 0xAA sync byte
      READ_CMD,     // Next byte tells us what command this is
      READ_LEN,     // Next byte tells us how many payload bytes follow
      READ_DATA,    // Read 'len' bytes of payload
      READ_CRC      // Final byte: checksum (XOR of all previous)
    };

    State state;

    uint8_t temp_cmd_;
    uint8_t temp_len_;
    uint8_t checksum_;
    size_t index_;
    std::vector<uint8_t> data_buf_;

    // Objects for logging
    rclcpp::Logger logger_;

};

#endif //SERIAL_COMM_H
