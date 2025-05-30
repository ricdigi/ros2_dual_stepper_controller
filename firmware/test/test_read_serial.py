import serial
import struct
import time

HEADER = 0xAA
VEL_DATA_LEN = 8
WAIT_HEADER, READ_CMD, READ_LEN, READ_DATA, READ_CRC = range(5)

def read_packet(ser):
    state = WAIT_HEADER
    checksum = 0
    cmd = 0
    length = 0
    data = bytearray()
    index = 0

    while True:
        if ser.in_waiting == 0:
            time.sleep(0.01)
            continue

        byte_in = ser.read(1)[0]

        if state == WAIT_HEADER:
            if byte_in == HEADER:
                checksum = byte_in
                state = READ_CMD

        elif state == READ_CMD:
            cmd = byte_in
            checksum ^= byte_in
            state = READ_LEN

        elif state == READ_LEN:
            length = byte_in
            if length > VEL_DATA_LEN:
                state = WAIT_HEADER
            else:
                checksum ^= byte_in
                data.clear()
                index = 0
                state = READ_DATA

        elif state == READ_DATA:
            data.append(byte_in)
            checksum ^= byte_in
            index += 1
            if index >= length:
                state = READ_CRC

        elif state == READ_CRC:
            if checksum == byte_in:
                return cmd, data
            else:
                print("Checksum mismatch, discarding packet")
                state = WAIT_HEADER

def main():
    port = '/dev/cu.usbserial-2140'  # adjust as needed
    baud = 115200

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return

    print(f"Listening on {port} at {baud} baud...")

    while True:
        packet = read_packet(ser)
        if packet:
            cmd, data = packet
            print(f"Received cmd: {cmd}, data bytes: {[hex(b) for b in data]}")
            if len(data) == 8:
                enc_a, enc_b = struct.unpack('<ff', data)
                print(f"Encoder A: {enc_a:.3f}, Encoder B: {enc_b:.3f}")

if __name__ == "__main__":
    main()

