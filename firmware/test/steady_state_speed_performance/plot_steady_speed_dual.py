#!/usr/bin/env python3
"""
Plot steady‑speed mean ± 1 σ for each motor in separate subplots.

Input : steady_speed_detailed.csv
Output: one figure, left subplot = motor A, right subplot = motor B
         – shaded ±1 σ
         – ideal y = x reference
"""

import csv, numpy as np, matplotlib.pyplot as plt

CSV_FILE = "steady_speed_detailed.csv"

# ── collect samples per command speed ───────────────────────────────────────
samples = {"A": {}, "B": {}}

with open(CSV_FILE, newline="") as f:
    for r in csv.DictReader(f):
        sp  = float(r["speed_cmd"])
        vA  = float(r["speed_A_rad_s"])
        vB  = float(r["speed_B_rad_s"])
        samples["A"].setdefault(sp, []).append(vA)
        samples["B"].setdefault(sp, []).append(vB)

if not samples["A"]:
    raise SystemExit("No data found in steady_speed_detailed.csv")

speeds = np.array(sorted(samples["A"]))        # same keys for both motors

def stats(motor_key):
    mu  = np.array([np.mean(samples[motor_key][s]) for s in speeds])
    sig = np.array([np.std (samples[motor_key][s], ddof=0) for s in speeds])
    return mu, sig

muA, sigA = stats("A")
muB, sigB = stats("B")

# ── plotting ───────────────────────────────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(12, 4), sharey=True)

# --- Motor A (left) ---
ax = axes[0]
ax.plot(speeds, muA, "o-", label="Mean A", color="tab:blue")
ax.fill_between(speeds, muA - sigA, muA + sigA, alpha=0.25, color="tab:blue",
                label="±1 σ A")
ax.plot(speeds, speeds, "k:",  label="Ideal y=x")   # 45° line
ax.set_title("Motor A (left)")
ax.set_xlabel("Command speed (rad/s)")
ax.set_ylabel("Measured speed (rad/s)")
ax.grid(True)
ax.legend(loc="upper left")

# --- Motor B (right) ---
ax = axes[1]
ax.plot(speeds, muB, "s--", label="Mean B", color="tab:red")
ax.fill_between(speeds, muB - sigB, muB + sigB, alpha=0.25, color="tab:red",
                label="±1 σ B")
ax.plot(speeds, speeds, "k:",  label="Ideal y=x")
ax.set_title("Motor B (right)")
ax.set_xlabel("Command speed (rad/s)")
ax.grid(True)
ax.legend(loc="upper left")

fig.tight_layout()
plt.show()
