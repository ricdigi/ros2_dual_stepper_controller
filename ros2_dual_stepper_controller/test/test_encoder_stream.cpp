#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include "ros2_dual_stepper_controller/serial_comm.hpp"

TEST(SerialCommHardwareTest, ReadStreamingEncoderPackets) {
    SerialComm comm;

    const char* port = std::getenv("TEST_SERIAL_PORT");
    ASSERT_NE(port, nullptr) << "Set TEST_SERIAL_PORT environment variable";
    comm.setSerialPort(port);

    comm.setBaudRate(115200);
    ASSERT_EQ(comm.init(), hardware_interface::CallbackReturn::SUCCESS);

    constexpr int expected_packet_count = 10;
    int received_count = 0;

    for (int i = 0; i < 200 && received_count < expected_packet_count; ++i) {
        uint8_t cmd;
        std::vector<uint8_t> data;

        if (comm.receive(cmd, data)) {
            if (cmd == SerialComm::ENC_CMD && data.size() == SerialComm::ENC_DATA_LEN) {
                float left, right;
                std::memcpy(&left, &data[0], 4);
                std::memcpy(&right, &data[4], 4);

                std::cout << "Left: " << left << " rad, Right: " << right << " rad" << std::endl;
                received_count++;
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    EXPECT_GE(received_count, expected_packet_count);
}
