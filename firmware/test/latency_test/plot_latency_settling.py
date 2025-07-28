#!/usr/bin/env python3
"""
Latency / settling plot for the two‑motor CSV produced by
dual_motor_latency_test.py

• Two stacked sub‑plots (latency, settling)
• Motor A and Motor B plotted together in each sub‑plot
• Mean curves with low‑opacity variance ribbons
• Works with or without header row
"""

import csv, sys, numpy as np, matplotlib.pyplot as plt

CSV_FILE = "latency_settling_full_log.csv"

# ──────────────────────── load & filter CSV ────────────────────────────────
lat_A, lat_B = {}, {}   # speed → list
set_A, set_B = {}, {}

with open(CSV_FILE, newline="") as f:
    first = next(csv.reader(f))
    header = first[0].lower().startswith("trial")
    f.seek(0)

    if header:
        rdr = csv.DictReader(f)
        for row in rdr:
            spd  = float(row["command_speed"])
            lA   = float(row["latency_A_s"])
            sA   = float(row["settling_A_s"])
            lB   = float(row["latency_B_s"])
            sB   = float(row["settling_B_s"])
            if lA >= 0: lat_A.setdefault(spd, []).append(lA)
            if sA >= 0: set_A.setdefault(spd, []).append(sA)
            if lB >= 0: lat_B.setdefault(spd, []).append(lB)
            if sB >= 0: set_B.setdefault(spd, []).append(sB)
    else:
        rdr = csv.reader(f)
        for row in rdr:
            # fixed order: trial, cmd, latA, setA, finA, latB, setB, finB
            _, spd, lA, sA, _, lB, sB, _ = map(float, row)
            if lA >= 0: lat_A.setdefault(spd, []).append(lA)
            if sA >= 0: set_A.setdefault(spd, []).append(sA)
            if lB >= 0: lat_B.setdefault(spd, []).append(lB)
            if sB >= 0: set_B.setdefault(spd, []).append(sB)

if not (lat_A or lat_B):
    sys.exit("No positive latency / settling values found.")

# ───────────────────────────── helper --------------------------------------
def build_series(dct):
    speeds = sorted(dct)
    mu  = np.array([np.mean(dct[s]) for s in speeds])
    var = np.array([np.var (dct[s]) for s in speeds])
    return speeds, mu, var

sA_lat, muA_lat, varA_lat = build_series(lat_A) if lat_A else ([],[],[])
sB_lat, muB_lat, varB_lat = build_series(lat_B) if lat_B else ([],[],[])
sA_set, muA_set, varA_set = build_series(set_A) if set_A else ([],[],[])
sB_set, muB_set, varB_set = build_series(set_B) if set_B else ([],[],[])

# ───────────────────────────── plotting -------------------------------------
fig, (ax_lat, ax_set) = plt.subplots(2, 1, sharex=False, figsize=(7, 8))

# latency
if sA_lat:
    ax_lat.plot(sA_lat, muA_lat, "o-", label="Latency A", color="tab:blue")
    ax_lat.fill_between(sA_lat, muA_lat-varA_lat, muA_lat+varA_lat,
                        color="tab:blue", alpha=0.2)
if sB_lat:
    ax_lat.plot(sB_lat, muB_lat, "s--", label="Latency B", color="tab:red")
    ax_lat.fill_between(sB_lat, muB_lat-varB_lat, muB_lat+varB_lat,
                        color="tab:red", alpha=0.2)
ax_lat.set_ylabel("Latency (s)")
ax_lat.set_title("Latency vs Command speed")
ax_lat.grid(True)
ax_lat.legend()

# settling
if sA_set:
    ax_set.plot(sA_set, muA_set, "o-", label="Settling A", color="tab:blue")
    ax_set.fill_between(sA_set, muA_set-varA_set, muA_set+varA_set,
                        color="tab:blue", alpha=0.2)
if sB_set:
    ax_set.plot(sB_set, muB_set, "s--", label="Settling B", color="tab:red")
    ax_set.fill_between(sB_set, muB_set-varB_set, muB_set+varB_set,
                        color="tab:red", alpha=0.2)
ax_set.set_xlabel("Command speed (rad/s)")
ax_set.set_ylabel("Settling time (s)")
ax_set.set_title("Settling vs Command speed")
ax_set.grid(True)
ax_set.legend()

plt.tight_layout()
plt.show()
