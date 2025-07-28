#!/usr/bin/env python3
"""
Max‑speed sweep: 0‑30 rad/s in 1 rad/s steps.
• Holds each command for HOLD_S seconds.
• Logs mean and σ of measured speed for both motors.
"""

import serial, struct, time, csv, numpy as np

# ───────── configuration ────────────────────────────────────────────────────
PORT = '/dev/cu.usbserial-1230';  BAUD = 115200
HEADER, VEL_CMD, ENC_CMD, ENC_LEN = 0xAA, 0x01, 0x02, 8

CMD_MIN, CMD_MAX, CMD_STEP = 0, 30, 1
HOLD_S   = 2.0            # time at each step
WARMUP_S = 2.0            # zero‑speed buffer fill
MAX_RAD  = 50.0           # clamp derivative

CSV_FIELDS = ["cmd_speed",
              "mean_A", "std_A",
              "mean_B", "std_B"]
# ────────────────────────────────────────────────────────────────────────────


def build_pkt(v_a, v_b):
    data = struct.pack('<ff', v_a, v_b)
    pkt  = bytearray([HEADER, VEL_CMD, len(data)]) + data
    chk  = 0
    for b in pkt: chk ^= b
    pkt.append(chk)
    return pkt


def read_enc(ser):
    st = cs = cmd = ln = 0; data = bytearray(); t0 = time.time()
    while time.time() - t0 < 0.1:
        if ser.in_waiting == 0:
            time.sleep(0.001); continue
        b = ser.read(1)[0]
        if   st == 0 and b == HEADER: cs = b; st = 1
        elif st == 1: cmd = b; cs ^= b; st = 2
        elif st == 2: ln = b; cs ^= b; data.clear(); st = 3
        elif st == 3:
            data.append(b); cs ^= b
            if len(data) == ln: st = 4
        elif st == 4:
            if cs == b and cmd == ENC_CMD and ln == ENC_LEN:
                return struct.unpack('<ff', data)      # degrees
            st = 0
    return None


def deriv(buf, prev, unwrap, ang):
    if prev is not None:
        d = ang - prev
        if   d > 180:  d -= 360
        elif d < -180: d += 360
        unwrap += d
    else:
        unwrap = ang
    prev = ang

    buf.append((time.time(), unwrap))
    if len(buf) > 3: buf.pop(0)

    if len(buf) < 3:
        return 0.0, prev, unwrap
    (t0,a0),(_, _),(t2,a2) = buf[-3:]
    dt = t2 - t0
    if dt <= 0: return 0.0, prev, unwrap
    v = np.clip(np.deg2rad((a2 - a0)/dt), -MAX_RAD, MAX_RAD)
    return v, prev, unwrap


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(2)

    # active warm‑up at 0 rad/s
    t0 = time.time()
    while time.time() - t0 < WARMUP_S:
        ser.write(build_pkt(0.0, 0.0)); read_enc(ser)

    results = []

    for cmd in range(CMD_MIN, CMD_MAX + 1, CMD_STEP):
        print(f"cmd={cmd} rad/s")

        start = time.time()
        stop  = start + HOLD_S

        buf  = [[], []]
        prev = [None, None]
        unwrap = [0.0, 0.0]
        vA, vB = [], []

        while time.time() < stop:
            ser.write(build_pkt(cmd, cmd))
            enc = read_enc(ser)
            if enc is None: continue

            v = [0.0, 0.0]
            for i in (0,1):
                v[i], prev[i], unwrap[i] = deriv(
                    buf[i], prev[i], unwrap[i], enc[i])
            vA.append(v[0]); vB.append(v[1])

        results.append({"cmd_speed": cmd,
                        "mean_A": float(np.mean(vA)),
                        "std_A":  float(np.std(vA, ddof=0)),
                        "mean_B": float(np.mean(vB)),
                        "std_B":  float(np.std(vB, ddof=0))})

    ser.write(build_pkt(0.0, 0.0)); ser.close()

    with open("max_speed_test.csv", "w", newline="") as f:
        csv.DictWriter(f, CSV_FIELDS).writerows(results)
    print("✔ max_speed_test.csv written")


if __name__ == "__main__":
    main()
