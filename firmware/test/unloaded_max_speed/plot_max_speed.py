#!/usr/bin/env python3
import csv, numpy as np, matplotlib.pyplot as plt

CSV_FILE = "max_speed_test.csv"
FIELDNAMES = ["cmd_speed", "mean_A", "std_A", "mean_B", "std_B"]

cmd, mA, sA, mB, sB = [], [], [], [], []

with open(CSV_FILE, newline="") as f:
    rdr = csv.reader(f)
    first = next(rdr)

    # Detect whether the first row is header or data
    has_header = first[0].lower().strip() in {"cmd_speed", "command_speed"}
    rows = rdr if has_header else [first] + list(rdr)

    for r in rows:
        cmd.append(float(r[0]))
        mA .append(float(r[1]));  sA .append(float(r[2]))
        mB .append(float(r[3]));  sB .append(float(r[4]))

sp = np.array(cmd)
mA = np.array(mA); sA = np.array(sA)
mB = np.array(mB); sB = np.array(sB)

fig, ax = plt.subplots(1, 2, figsize=(12, 4), sharey=True)

# Motor A
ax[0].plot(sp, mA, "o-", color="tab:blue", label="Mean A")
ax[0].fill_between(sp, mA - sA, mA + sA, alpha=0.25, color="tab:blue",
                   label="±1 σ")
ax[0].plot(sp, sp, "k:", label="Ideal y=x")
ax[0].set_title("Motor A")
ax[0].set_xlabel("Command speed (rad/s)")
ax[0].set_ylabel("Measured speed (rad/s)")
ax[0].grid(True); ax[0].legend(loc="upper left")

# Motor B
ax[1].plot(sp, mB, "s--", color="tab:red", label="Mean B")
ax[1].fill_between(sp, mB - sB, mB + sB, alpha=0.25, color="tab:red",
                   label="±1 σ")
ax[1].plot(sp, sp, "k:", label="Ideal y=x")
ax[1].set_title("Motor B")
ax[1].set_xlabel("Command speed (rad/s)")
ax[1].grid(True); ax[1].legend(loc="upper left")

fig.tight_layout()
plt.show()
