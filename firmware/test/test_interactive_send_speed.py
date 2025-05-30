import serial
import struct
import time

PORT = '/dev/cu.usbserial-2140'
BAUD = 115200

def build_packet(speed_a, speed_b):
    HEADER = 0xAA
    CMD = 0x01
    data = struct.pack('<ff', speed_a, speed_b)  # two 4-byte floats
    LEN = len(data)  # should be 8

    packet = bytearray([HEADER, CMD, LEN]) + data
    checksum = 0
    for b in packet:
        checksum ^= b
    packet.append(checksum)
    return packet

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)  # allow Arduino to reset

        print("Enter motor A and B speeds in rad/s (e.g. 3.0 -3.0), Ctrl+C to quit:")
        while True:
            try:
                user_input = input("> ")
                parts = user_input.strip().split()
                if len(parts) != 2:
                    print("⚠️ Enter two numbers separated by space.")
                    continue
                speed_a = float(parts[0])
                speed_b = float(parts[1])
                packet = build_packet(speed_a, speed_b)
                ser.write(packet)
                print(f"✅ Sent A={speed_a}, B={speed_b} → {list(packet)}")

                
            except ValueError:
                print("⚠️ Please enter valid numbers.")
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()

