
#include "ros2_dual_stepper_controller/serial_comm.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>


void SerialComm::setSerialPort(const std::string & serial_port) {
    serial_port_ = serial_port;
}


void SerialComm::setBaudRate(int baud_rate) {
    baud_rate_ = baud_rate;
}


hardware_interface::CallbackReturn SerialComm::init() {

    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("Hardware"), "Failed to open serial port: " << strerror(errno));
        return hardware_interface::CallbackReturn::ERROR;
    }

    struct termios tty {};
    if (tcgetattr(serial_fd_, &tty) != 0) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("Hardware"), "tcgetattr failed: " << strerror(errno));
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Map int baud_rate_ to termios constant
    speed_t baud;
    if (baud_rate_ == 9600) baud = B9600;
    else if (baud_rate_ == 19200) baud = B19200;
    else if (baud_rate_ == 38400) baud = B38400;
    else if (baud_rate_ == 57600) baud = B57600;
    else if (baud_rate_ == 115200) baud = B115200;
    else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("Hardware"), "Unsupported baud rate");
        return hardware_interface::CallbackReturn::ERROR;
    }

    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 0;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("Hardware"), "tcsetattr failed: " << strerror(errno));
        return hardware_interface::CallbackReturn::ERROR;
    }

    state = WAIT_HEADER;
    index = 0;
    data_buf_.reserve(MAX_PACKET_SIZE);

    return hardware_interface::CallbackReturn::SUCCESS;

}


bool SerialComm::receive(uint8_t &cmd, std::vector<uint8_t> &data) {
    uint8_t byteIn;
    ssize_t n = read(serial_fd_, &byteIn, 1);
    if (n <= 0) return false;

    switch (state) {
        case WAIT_HEADER:
            if (byteIn == HEADER) {
                checksum_ = byteIn;
                state = READ_CMD;
            }
            break;

        case READ_CMD:
            temp_cmd_ = byteIn;
            checksum_ ^= byteIn;
            state = READ_LEN;
            break;

        case READ_LEN:
            temp_len_ = byteIn;
            if (temp_len_ > MAX_PACKET_SIZE) {
                state = WAIT_HEADER;
            } else {
                checksum_ ^= byteIn;
                data_buf_.clear();
                data_buf_.resize(temp_len_, 0);
                index_ = 0;
                state = READ_DATA;
            }
            break;

        case READ_DATA:
            data_buf_[index_++] = byteIn;
            checksum_ ^= byteIn;
            if (index_ >= temp_len_) {
                state = READ_CRC;
            }
            break;

        case READ_CRC:
            state = WAIT_HEADER;
            if (checksum_ == byteIn) {
                cmd = temp_cmd_;
                data = data_buf_;
                return true;
            }
            break;

        default:
            state = WAIT_HEADER;
            break;
    }

    return false;
}


bool SerialComm::send(uint8_t cmd, const std::vector<uint8_t> &data) {
    if (data.size() > MAX_PACKET_SIZE) return false;

    uint8_t len = static_cast<uint8_t>(data.size());
    uint8_t checksum = HEADER ^ cmd ^ len;

    std::vector<uint8_t> packet;
    packet.reserve(3 + len + 1);

    packet.push_back(HEADER);
    packet.push_back(cmd);
    packet.push_back(len);

    for (uint8_t byte : data) {
        packet.push_back(byte);
        checksum ^= byte;
    }

    packet.push_back(checksum);

    ssize_t bytes_written = write(serial_fd_, packet.data(), packet.size());
    return bytes_written == static_cast<ssize_t>(packet.size());
}



