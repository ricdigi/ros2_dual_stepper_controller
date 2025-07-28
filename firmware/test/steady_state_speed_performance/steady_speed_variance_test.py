#!/usr/bin/env python3
"""
Steady‑state speed sweep with live output.

• 2 s active warm‑up at zero speed (fills encoder buffers).
• Live print of t, cmd, instantaneous speeds.
• Speed calculation uses angle unwrapping + 3‑point central diff.
"""

import serial, struct, time, csv, numpy as np

# ───────── configuration ────────────────────────────────────────────────────
PORT   = '/dev/cu.usbserial-1230';  BAUD = 115200
HEADER, VEL_CMD, ENC_CMD, ENC_LEN = 0xAA, 0x01, 0x02, 8

TARGET_SPEEDS = list(range(1, 16))     # 1 … 20 rad/s
REPEAT        = 1
WARMUP_S      = 2.0                    # active warm‑up ↓
MEASURE_S     = 4.0
MAX_RAD       = 20.0

SUM_FIELDS = ["speed_cmd", "mean_A", "var_A", "mean_B", "var_B"]
DET_FIELDS = ["speed_cmd", "t_rel_s", "speed_A_rad_s", "speed_B_rad_s"]
# ────────────────────────────────────────────────────────────────────────────


def build_pkt(v_a, v_b):
    data = struct.pack('<ff', v_a, v_b)
    pkt  = bytearray([HEADER, VEL_CMD, len(data)]) + data
    chk  = 0
    for b in pkt: chk ^= b
    pkt.append(chk)
    return pkt


def read_enc(ser):
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
                return struct.unpack('<ff', data)      # degrees
            state = 0
    return None


# ───────── speed calculation helpers ────────────────────────────────────────
def compute_speed(buf, prev_ang, unwrap, ang_new):
    """Return (v_rad_s, prev_ang, unwrap)."""
    if prev_ang is not None:
        d = ang_new - prev_ang
        if   d > 180:  d -= 360
        elif d < -180: d += 360
        unwrap += d
    else:
        unwrap = ang_new
    prev_ang = ang_new

    buf.append((time.time(), unwrap))
    if len(buf) > 3:
        buf.pop(0)

    if len(buf) < 3:
        return 0.0, prev_ang, unwrap

    (t0, a0), (_, _), (t2, a2) = buf[-3:]
    dt = t2 - t0
    if dt <= 0:
        return 0.0, prev_ang, unwrap
    v = np.clip(np.deg2rad((a2 - a0) / dt), -MAX_RAD, MAX_RAD)
    return v, prev_ang, unwrap
# ────────────────────────────────────────────────────────────────────────────


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(2)                                     # USB‑serial settle

    summary, detailed = [], []

    for spd in TARGET_SPEEDS:
        print(f"\n=== Command {spd} rad/s ===")
        for trial in range(REPEAT):

            # ---------- 2 s active warm‑up at 0 rad/s ----------
            t0 = time.time()
            while time.time() - t0 < WARMUP_S:
                ser.write(build_pkt(0.0, 0.0))
                read_enc(ser)                         # flush junk

            # ---------- measurement window ----------
            start = time.time()
            stop  = start + MEASURE_S

            # per‑motor state
            buf  = [[], []]        # 0→A, 1→B
            prev = [None, None]
            unwrap = [0.0, 0.0]
            vA, vB = [], []

            while time.time() < stop:
                now = time.time()
                t_rel = now - start
                ser.write(build_pkt(spd, spd))        # keep‑alive

                enc = read_enc(ser)
                if enc is None:
                    continue

                speeds = [0.0, 0.0]                   # temp print holder
                for i in (0, 1):
                    v, prev[i], unwrap[i] = compute_speed(
                        buf[i], prev[i], unwrap[i], enc[i])
                    speeds[i] = v
                vA.append(speeds[0]); vB.append(speeds[1])

                detailed.append({"speed_cmd": spd,
                                 "t_rel_s": t_rel,
                                 "speed_A_rad_s": speeds[0],
                                 "speed_B_rad_s": speeds[1]})

                # -------- live print -------------
                print(f"t={t_rel:5.3f}s  cmd={spd:.2f} "
                      f"A={speeds[0]:.2f}  B={speeds[1]:.2f}")

            # ---------- per‑trial stats ----------
            summary.append({"speed_cmd": spd,
                            "mean_A": np.mean(vA), "var_A": np.var(vA),
                            "mean_B": np.mean(vB), "var_B": np.var(vB)})

            # inter‑trial idle
            ser.write(build_pkt(0.0, 0.0))
            time.sleep(0.5)

    ser.write(build_pkt(0.0, 0.0)); ser.close()

    # ---------- CSV output ----------
    with open("steady_speed_summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, SUM_FIELDS); w.writeheader(); w.writerows(summary)
    with open("steady_speed_detailed.csv", "w", newline="") as f:
        w = csv.DictWriter(f, DET_FIELDS); w.writeheader(); w.writerows(detailed)
    print("✔ Logs written: steady_speed_summary.csv, steady_speed_detailed.csv")


if __name__ == "__main__":
    main()
