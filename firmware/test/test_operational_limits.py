import serial
import struct
import time
import numpy as np
import csv
import matplotlib.pyplot as plt

PORT = '/dev/cu.usbserial-2140'
BAUD = 115200
HEADER = 0xAA
VEL_CMD = 0x01
ENC_CMD = 0x02
VEL_DATA_LEN = 8
ENC_DATA_LEN = 8

def build_velocity_packet(speed_a, speed_b):
    data = struct.pack('<ff', speed_a, speed_b)
    packet = bytearray([HEADER, VEL_CMD, len(data)]) + data
    checksum = 0
    for b in packet:
        checksum ^= b
    packet.append(checksum)
    return packet

def read_encoder_packet(ser):
    state = 0
    checksum = 0
    cmd = 0
    length = 0
    data = bytearray()
    start = time.time()

    while time.time() - start < 0.1:  # max 100 ms
        if ser.in_waiting == 0:
            time.sleep(0.001)
            continue

        byte_in = ser.read(1)
        if not byte_in:
            continue
        byte_in = byte_in[0]

        if state == 0:  # WAIT_HEADER
            if byte_in == HEADER:
                checksum = byte_in
                state = 1
        elif state == 1:  # READ_CMD
            cmd = byte_in
            checksum ^= byte_in
            state = 2
        elif state == 2:  # READ_LEN
            length = byte_in
            checksum ^= byte_in
            data = bytearray()
            state = 3
        elif state == 3:  # READ_DATA
            data.append(byte_in)
            checksum ^= byte_in
            if len(data) == length:
                state = 4
        elif state == 4:  # READ_CRC
            if checksum == byte_in and cmd == ENC_CMD and length == ENC_DATA_LEN:
                enc_a, enc_b = struct.unpack('<ff', data)
                #ser.reset_input_buffer()
                return enc_a, enc_b
            else:
                state = 0


    print("Timeout waiting for encoder packet")
    return None  # timed out


def unwrap_angle(prev, curr):
    delta = curr - prev
    if abs(delta) > 180.0:
        if delta > 0:
            delta -= 360.0
        else:
            delta += 360.0
    return delta


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)

    time_between_speeds = 0.3 # seconds
    previous_time = 0
    index = 0

    speeds = np.concatenate([
        np.arange(0, 5.0, 0.1),     # ramp up
        np.arange(5.0, -0.1, -0.1)  # ramp down
    ])

    results_a = []
    results_b = []

    enc_a = 0.0
    enc_b = 0.0

    prev_a = None
    prev_b = None

    abs_a = 0.0
    abs_b = 0.0

    print("Starting speed sweep test...")

    time.sleep(2)

    while True and index < len(speeds):

        if time.time() - previous_time > time_between_speeds:
            cmd_speed = speeds[index]
            packet = build_velocity_packet(cmd_speed, cmd_speed)
            ser.write(packet)
            previous_time = time.time()
            index += 1
            print(f"Commanded Speed: {cmd_speed:.2f} rad/s")


        result = read_encoder_packet(ser)
        if result is not None:
            enc_a, enc_b = result

        # Inside main loop:
        timestamp = time.time()

        if enc_a != prev_a:
            abs_a += unwrap_angle(prev_a, enc_a) if prev_a is not None else 0
            results_a.append((timestamp, cmd_speed, enc_a, abs_a))
            prev_a = enc_a

        if enc_b != prev_b:
            abs_b += unwrap_angle(prev_b, enc_b) if prev_b is not None else 0
            results_b.append((timestamp, cmd_speed, enc_b, abs_b))
            prev_b = enc_b

        print(f"Encoder A: {enc_a:.2f}, Encoder B: {enc_b:.2f}, Abs A: {abs_a:.2f}, Abs B: {abs_b:.2f}")

    ser.close()

    print("Test complete, plotting results...")

    # Convert to NumPy arrays
    results_a = np.array(results_a)
    results_b = np.array(results_b)

    # Extract columns
    times_a = results_a[:, 0] - results_a[0, 0]
    cmds_a = results_a[:, 1]
    pos_a_deg = results_a[:, 3]
    pos_a_rad = np.deg2rad(pos_a_deg)
    speed_a = np.gradient(pos_a_rad, times_a)

    times_b = results_b[:, 0] - results_b[0, 0]
    cmds_b = results_b[:, 1]
    pos_b_deg = results_b[:, 3]
    pos_b_rad = np.deg2rad(pos_b_deg)
    speed_b = np.gradient(pos_b_rad, times_b)


    plt.figure()
    plt.plot(times_a, cmds_a, label='Commanded Speed A', linewidth=1.5)
    plt.plot(times_b, cmds_b, label='Commanded Speed B', linewidth=1.5)
    plt.plot(times_a, speed_a, label='Measured Speed A', linestyle='--')
    plt.plot(times_b, speed_b, label='Measured Speed B', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (rad/s)')
    plt.title('Commanded vs Measured Speeds (A and B)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()













if __name__ == "__main__":
    main()
