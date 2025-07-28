#!/usr/bin/env python3
"""
Latency / settling tester for TWO motors (channels A & B).

• Sends identical commands to both channels.
• Logs per‑sample speeds for A & B in the same row.
• Logs per‑trial latency / settling separately for A & B.
• Prints live speeds for both motors, including warm‑up phase.
"""

import serial, struct, time, csv, numpy as np, matplotlib.pyplot as plt

# ───────── configuration ────────────────────────────────────────────────────
PORT = '/dev/cu.usbserial-1230'; BAUD = 115200
HEADER, VEL_CMD, ENC_CMD, ENC_LEN = 0xAA, 0x01, 0x02, 8

BASE_SPEEDS       = [1, 2, 3, 4, 5]   # rad/s
REPEAT            = 10                # trials per speed
WARMUP_DURATION   = 2.0               # s
MEASUREMENT_TIME  = 3.0               # s  (after command)
SETTLING_DURATION = 0.5               # s
TOLERANCE         = 0.10              # ±10 %
ANGLE_THRESH      = 0.5               # deg
MAX_SPEED_RAD     = 20.0              # clip

DETAILED_FIELDS = ["trial", "command_speed",
                   "t_rel_s", "speed_A_rad_s", "speed_B_rad_s"]
SUMMARY_FIELDS  = ["trial", "command_speed",
                   "latency_A_s",  "settling_A_s",  "final_speed_A_rad_s",
                   "latency_B_s",  "settling_B_s",  "final_speed_B_rad_s"]
# ────────────────────────────────────────────────────────────────────────────


def build_velocity_packet(v_a, v_b):
    data = struct.pack('<ff', v_a, v_b)
    pkt  = bytearray([HEADER, VEL_CMD, len(data)]) + data
    chk  = 0
    for b in pkt: chk ^= b
    pkt.append(chk)
    return pkt

def read_encoder_packet(ser):
    state = checksum = cmd = length = 0; data = bytearray(); t0 = time.time()
    while time.time() - t0 < 0.1:
        if ser.in_waiting == 0:
            time.sleep(0.001); continue
        b = ser.read(1)[0]
        if   state == 0 and b == HEADER: checksum = b; state = 1
        elif state == 1: cmd = b; checksum ^= b; state = 2
        elif state == 2: length = b; checksum ^= b; data.clear(); state = 3
        elif state == 3:
            data.append(b); checksum ^= b
            if len(data) == length: state = 4
        elif state == 4:
            if checksum == b and cmd == ENC_CMD and length == ENC_LEN:
                return struct.unpack('<ff', data)      # enc_a, enc_b (deg)
            state = 0
    return None


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(2)                                      # port settle

    results, detailed = [], []

    print("=== Dual‑motor latency / settling test ===")

    for cmd_speed in BASE_SPEEDS:
        for trial in range(1, REPEAT + 1):
            print(f"\n--- Trial {trial} | target {cmd_speed:.2f} rad/s ---")

            # 0‑speed warm‑up (fills encoder buffers)
            t0 = time.time()
            while time.time() - t0 < WARMUP_DURATION:
                ser.write(build_velocity_packet(0.0, 0.0))
                read_encoder_packet(ser)

            cmd_time  = time.time()
            stop_time = cmd_time + MEASUREMENT_TIME

            # per‑motor state (idx 0=A, 1=B)
            buf, prev, unwrap = [[], []], [None, None], [0.0, 0.0]
            latency = [None, None]; settling = [None, None]
            settle_start = [None, None]; final_speed = [None, None]

            while time.time() < stop_time:
                now      = time.time()
                t_rel    = now - cmd_time          # negative during warm‑up
                cur_cmd  = 0.0 if now < cmd_time else cmd_speed
                ser.write(build_velocity_packet(cur_cmd, cur_cmd))

                enc = read_encoder_packet(ser)
                if enc is None:
                    continue

                speeds_rad = [0.0, 0.0]            # temp holder for row

                for i in (0, 1):                   # 0→A, 1→B
                    ang = enc[i]
                    # unwrap
                    if prev[i] is not None:
                        d = ang - prev[i]
                        if   d > 180: d -= 360
                        elif d < -180: d += 360
                        unwrap[i] += d
                    else:
                        unwrap[i] = ang
                    prev[i] = ang

                    buf[i].append((now, unwrap[i]))
                    if len(buf[i]) < 3:
                        continue
                    (t0_,a0),(t1_,a1),(t2_,a2) = buf[i][-3:]
                    dt = t2_ - t0_
                    if dt <= 0:
                        continue
                    v_rad = np.clip(np.deg2rad((a2 - a0) / dt),
                                    -MAX_SPEED_RAD, MAX_SPEED_RAD)
                    speeds_rad[i] = v_rad
                    final_speed[i] = v_rad

                    # latency / settling only after command
                    if t1_ < cmd_time:
                        continue
                    # latency
                    if latency[i] is None and abs(a1 - a0) > ANGLE_THRESH:
                        latency[i] = t1_ - cmd_time
                        print(f"  Motor {['A','B'][i]} latency {latency[i]:.3f}s")
                    # settling
                    if latency[i] is not None and settling[i] is None:
                        if abs(v_rad - cmd_speed) / cmd_speed <= TOLERANCE:
                            if settle_start[i] is None:
                                settle_start[i] = t1_
                            elif t1_ - settle_start[i] >= SETTLING_DURATION:
                                settling[i] = t1_ - cmd_time
                                print(f"  Motor {['A','B'][i]} settled "
                                      f"{settling[i]:.3f}s")
                        else:
                            settle_start[i] = None

                # ------ log (row for both motors) --------------------------
                detailed.append({"trial": trial,
                                 "command_speed": cur_cmd,
                                 "t_rel_s": t_rel,
                                 "speed_A_rad_s": speeds_rad[0],
                                 "speed_B_rad_s": speeds_rad[1]})

                # live print once per iteration
                print(f"t={t_rel:+5.3f}s  cmd={cur_cmd:.2f} "
                      f"A={speeds_rad[0]:.2f}  B={speeds_rad[1]:.2f}")

            # stop motors, cool‑down
            ser.write(build_velocity_packet(0.0, 0.0))
            time.sleep(0.5)

            results.append({
                "trial": trial, "command_speed": cmd_speed,
                "latency_A_s":  latency[0]  if latency[0]  is not None else -1,
                "settling_A_s": settling[0] if settling[0] is not None else -1,
                "final_speed_A_rad_s": final_speed[0] if final_speed[0] else -1,
                "latency_B_s":  latency[1]  if latency[1]  is not None else -1,
                "settling_B_s": settling[1] if settling[1] is not None else -1,
                "final_speed_B_rad_s": final_speed[1] if final_speed[1] else -1
            })
            time.sleep(1)

    ser.close()

    # save logs
    with open("latency_settling_full_log.csv", "w", newline="") as f:
        w = csv.DictWriter(f, SUMMARY_FIELDS); w.writeheader(); w.writerows(results)
    with open("velocity_detailed_log.csv", "w", newline="") as f:
        w = csv.DictWriter(f, DETAILED_FIELDS); w.writeheader(); w.writerows(detailed)
    print("\n✔ Logs written for both motors.")

    # quick plot for motor A
    a_rows = [r for r in results if r["latency_A_s"] >= 0]
    if a_rows:
        spd = [r["command_speed"] for r in a_rows]
        lat = [r["latency_A_s"]   for r in a_rows]
        stl = [r["settling_A_s"]  for r in a_rows]
        plt.figure()
        plt.plot(spd, lat, 'o-', label="Latency A")
        plt.plot(spd, stl, 's-', label="Settling A")
        plt.xlabel("Command speed (rad/s)")
        plt.ylabel("Time (s)")
        plt.title("Motor A latency / settling")
        plt.grid(); plt.legend(); plt.tight_layout(); plt.show()

if __name__ == "__main__":
    main()
