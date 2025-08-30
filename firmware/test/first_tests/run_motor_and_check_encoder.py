import serial
import struct
import time
import matplotlib.pyplot as plt
import numpy as np

PORT = '/dev/cu.usbserial-1230'
BAUD = 115200
HEADER = 0xAA
VEL_CMD = 0x01
ENC_CMD = 0x02

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

    while time.time() - start < 0.1:
        if ser.in_waiting == 0:
            time.sleep(0.001)
            continue

        byte_in = ser.read(1)
        if not byte_in:
            continue
        byte_in = byte_in[0]

        if state == 0:
            if byte_in == HEADER:
                checksum = byte_in
                state = 1
        elif state == 1:
            cmd = byte_in
            checksum ^= byte_in
            state = 2
        elif state == 2:
            length = byte_in
            checksum ^= byte_in
            data = bytearray()
            state = 3
        elif state == 3:
            data.append(byte_in)
            checksum ^= byte_in
            if len(data) == length:
                state = 4
        elif state == 4:
            if checksum == byte_in and cmd == ENC_CMD and length == 8:
                enc_a, enc_b = struct.unpack('<ff', data)
                return enc_a
            else:
                state = 0
    return None

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(1)

    target_speed = 1.0  # rad/s
    warmup_duration = 2.0  # seconds
    total_duration = 10.0  # seconds

    start_time = time.time()
    timestamps = []
    angles_unwrapped = []
    velocities_rad_per_s = []

    last_angle = None
    last_time = None
    total_angle = 0.0

    buffer = []  # stores tuples: (timestamp, unwrapped_angle)

    while True:
        now = time.time()
        elapsed = now - start_time
        if elapsed >= total_duration:
            break

        speed_cmd = 0.0 if elapsed < warmup_duration else target_speed
        packet = build_velocity_packet(speed_cmd, speed_cmd)
        ser.write(packet)

        enc = read_encoder_packet(ser)
        if enc is None:
            continue

        timestamp = elapsed
        angle = enc

        # Angle unwrapping
        if last_angle is not None:
            delta = angle - last_angle
            if delta > 180:
                delta -= 360
            elif delta < -180:
                delta += 360
            total_angle += delta
        else:
            total_angle = angle
        last_angle = angle

        # Save current point to buffer
        buffer.append((timestamp, total_angle))
        if len(buffer) < 3:
            continue  # need 3 points for central difference

        # Compute central difference
        t0, a0 = buffer[0]
        t1, a1 = buffer[1]
        t2, a2 = buffer[2]

        dt = t2 - t0
        if dt == 0:
            velocity_rad = 0.0
        else:
            v_deg = (a2 - a0) / dt
            velocity_rad = np.deg2rad(v_deg)

        # Save result at central point
        timestamps.append(t1)
        angles_unwrapped.append(a1)
        velocities_rad_per_s.append(velocity_rad)

        print(f"t={t1:.3f}s, angle={a1:.2f} deg, vel={velocity_rad:.2f} rad/s")

        # Drop oldest point
        buffer.pop(0)


# Stop motor
    packet = build_velocity_packet(0.0, 0.0)
    ser.write(packet)
    time.sleep(0.5)
    ser.close()

    # Plot angle
    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(timestamps, angles_unwrapped, label="Unwrapped Angle (deg)")
    plt.axvline(warmup_duration, color='gray', linestyle='--', label='Motor Start')
    plt.ylabel("Angle (deg)")
    plt.legend()
    plt.grid(True)

    # Plot velocity
    plt.subplot(2, 1, 2)
    plt.plot(timestamps, velocities_rad_per_s, label="Estimated Velocity (rad/s)")
    plt.axvline(warmup_duration, color='gray', linestyle='--')
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (rad/s)")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()



if __name__ == "__main__":
    main()
