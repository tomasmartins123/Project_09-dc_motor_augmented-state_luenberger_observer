# Project CRIA 08 - State Observer & Controller Validation Plotter
# Loads state observer CSV data and generates a high-resolution 3-panel plot 
# comparing speed tracking, disturbance estimations, and control signals.

import matplotlib.pyplot as plt
import pandas as pd

# ==========================================
# 1. Configuration
# ==========================================
CSV_FILE = "observer_validation_data.csv"
OUTPUT_IMAGE = "observer_validation_plot.png"

# ==========================================
# 2. Data Loading & Processing
# ==========================================
print(f"Loading validation data from '{CSV_FILE}'...")
data = pd.read_csv(CSV_FILE)

# Convert time from milliseconds to seconds
data["time_sec"] = data["time_ms"] / 1000.0

# Convert exact CSV columns to NumPy arrays
time_sec = data["time_sec"].to_numpy()

rpm_L_real = data["rpm_L_real"].to_numpy()
omega_hat_L = data["omega_hat_L"].to_numpy()
d_hat_L = data["d_hat_L"].to_numpy()
u_L = data["u_L"].to_numpy()

rpm_R_real = data["rpm_R_real"].to_numpy()
omega_hat_R = data["omega_hat_R"].to_numpy()
d_hat_R = data["d_hat_R"].to_numpy()
u_R = data["u_R"].to_numpy()

# ==========================================
# 3. Plotting
# ==========================================
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

# --- Subplot 1: Speed Tracking & Estimation ---
ax1.scatter(
    time_sec,
    rpm_L_real,
    color="#0072BD",
    alpha=0.3,
    s=15,
    label="Encoder Measurement (Left)",
)
ax1.plot(
    time_sec,
    omega_hat_L,
    color="#0072BD",
    linewidth=2.0,
    label=r"State Estimate $\hat{\omega}_L$",
)
ax1.scatter(
    time_sec,
    rpm_R_real,
    color="#D95319",
    alpha=0.3,
    s=15,
    label="Encoder Measurement (Right)",
)
ax1.plot(
    time_sec,
    omega_hat_R,
    color="#D95319",
    linewidth=2.0,
    label=r"State Estimate $\hat{\omega}_R$",
)
ax1.set_title("Speed Tracking vs. State Observer Estimation", fontweight="bold")
ax1.set_ylabel("Speed [RPM]", fontweight="bold")
ax1.grid(True, linestyle="--", alpha=0.5)
ax1.legend(loc="lower right")

# --- Subplot 2: Disturbance Estimation ---
ax2.plot(
    time_sec,
    d_hat_L,
    color="#0072BD",
    linewidth=2.0,
    label=r"Disturbance Estimate $\hat{d}_L$",
)
ax2.plot(
    time_sec,
    d_hat_R,
    color="#D95319",
    linewidth=2.0,
    label=r"Disturbance Estimate $\hat{d}_R$",
)
ax2.set_title("External Disturbance Estimation", fontweight="bold")
ax2.set_ylabel(r"Disturbance $\hat{d}$", fontweight="bold")
ax2.grid(True, linestyle="--", alpha=0.5)
ax2.legend(loc="upper right")

# --- Subplot 3: Control Action ---
ax3.plot(
    time_sec,
    u_L,
    color="#0072BD",
    linewidth=2.0,
    label="Control Output $u_L$",
)
ax3.plot(
    time_sec,
    u_R,
    color="#D95319",
    linewidth=2.0,
    label="Control Output $u_R$",
)
ax3.set_title("Control Signals Applied to Motors", fontweight="bold")
ax3.set_xlabel("Time [s]", fontweight="bold")
ax3.set_ylabel("Control Input $u$", fontweight="bold")
ax3.grid(True, linestyle="--", alpha=0.5)
ax3.legend(loc="lower right")

# Title and Layout Adjustments
fig.suptitle(
    "State Observer & Disturbance Rejection Performance Evaluation",
    fontsize=14,
    fontweight="bold",
)
plt.tight_layout()

# Save image and render
plt.savefig(OUTPUT_IMAGE, dpi=300)
print(f"Plot saved successfully as '{OUTPUT_IMAGE}'.")
plt.show()