#ifndef SERIALCOMM_H
#define SERIALCOMM_H

#include <stdint.h>

/*
    This class manages the communication between the motor controller board and the devices it connects to. The system is designed to stream out encoder data from two motors,
    and receive velocity commands to control them. This bi-directional communication happens via USB-serial, and the data is sent and received using structured binary packets.

    The structure of the binary packet is:

        [HEADER][CMD][LEN][DATA...][CHECKSUM]

    - HEADER:     1 byte, fixed value (e.g. 0xAA) used to mark the start of a new packet
    - CMD:        1 byte, command ID indicating the purpose of the packet (e.g. 0x01 for set velocity)
    - LEN:        1 byte, number of bytes in the DATA section
    - DATA:       variable length, payload depending on the CMD (e.g. two 4-byte floats for motor velocities)
    - CHECKSUM:   1 byte, XOR of all previous bytes (HEADER through last byte of DATA), used for integrity verification

    The receive parser is stateful and non-blocking, allowing integration into the main loop without disrupting timing-sensitive tasks.
    Upon successful reception and checksum validation, the decoded command can be retrieved and executed by the motor control logic.
*/

class SerialComm {

  public:
    SerialComm();
    void begin(long baud);

    bool receive();              
    bool hasCommand();  
    
    void sendEncoderData(float enc_a_data, float enc_b_data);

    uint8_t getCommand();
    float   getSpeedA();            // Extract motor A speed
    float   getSpeedB();            // Extract motor B speed

    void clearCommand();

    // Variables relative to the comunnication protocol
    inline static const uint8_t HEADER = 0xAA;

    inline static const uint8_t VEL_CMD = 0x01;
    inline static const uint8_t ENC_CMD = 0x02;
    inline static const uint8_t ENABLE_CMD = 0x03;
    inline static const uint8_t DISABLE_CMD = 0x04;

    inline static const uint8_t VEL_DATA_LEN = 8;
    inline static const uint8_t ENC_DATA_LEN = 8;
    inline static const uint8_t ENABLE_DATA_LEN = 1;

  private:

    enum State {
      WAIT_HEADER,  // Looking for the 0xAA sync byte
      READ_CMD,     // Next byte tells us what command this is
      READ_LEN,     // Next byte tells us how many payload bytes follow
      READ_DATA,    // Read 'len' bytes of payload
      READ_CRC      // Final byte: checksum (XOR of all previous)
    };

    State state;

    uint8_t cmd;
    uint8_t len;
    uint8_t data[VEL_DATA_LEN];
    uint8_t index;
    uint8_t checksum;

    bool commandReady;

};


#endif
