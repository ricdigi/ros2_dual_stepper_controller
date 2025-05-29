#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <cstring>
#include "ros2_dual_stepper_controller/serial_comm.hpp"


TEST(SerialCommHardwareTest, SendVelocityPacket) {
    SerialComm comm;

    const char* port = std::getenv("TEST_SERIAL_PORT");
    ASSERT_NE(port, nullptr) << "Set TEST_SERIAL_PORT environment variable";
    comm.setSerialPort(port);
    comm.setBaudRate(115200);
    ASSERT_EQ(comm.init(), hardware_interface::CallbackReturn::SUCCESS);

    // Wait for the serial sender (e.g. MCU) to boot and start transmitting
    std::this_thread::sleep_for(std::chrono::seconds(3));

    float left_speed = 1.0f;   // rad/s
    float right_speed = 1.0f;

    std::vector<uint8_t> data(8);
    memcpy(&data[0], &left_speed, 4);
    memcpy(&data[4], &right_speed, 4);

    bool result = comm.send(SerialComm::VEL_CMD, data);
    EXPECT_TRUE(result) << "Failed to send velocity command";

    // Optional: wait and then read back encoder if you expect visible change
    std::this_thread::sleep_for(std::chrono::seconds(2));
}
