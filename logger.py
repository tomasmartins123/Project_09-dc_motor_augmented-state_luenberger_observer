# Project CRIA 08 - Automatic Serial Data Logger for State Observer
# Logs state observer telemetry and control signals from Arduino to 'observer_validation_data.csv'.

import time
import serial

# ==========================================
# Configuration
# ==========================================
SERIAL_PORT = "COM3"
BAUD_RATE = 115200
OUTPUT_FILE = "observer_validation_data.csv"
DURATION_SEC = 20.0

# ==========================================
# Serial Connection & Logging
# ==========================================
try:
    print(f"Connecting to Arduino on {SERIAL_PORT} @ {BAUD_RATE} baud...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Wait for Arduino auto-reset

    print(
        f"Logging data to '{OUTPUT_FILE}' for {DURATION_SEC} seconds...\n"
    )

    start_time = time.time()
    lines_written = 0

    with open(OUTPUT_FILE, "w", encoding="utf-8") as file:
        while (time.time() - start_time) < DURATION_SEC:
            if ser.in_waiting > 0:
                raw_line = (
                    ser.readline().decode("utf-8", errors="ignore").strip()
                )

                if raw_line:
                    file.write(raw_line + "\n")
                    lines_written += 1
                    print(raw_line)

    ser.close()
    print(f"\nSuccess! {lines_written} lines recorded to '{OUTPUT_FILE}'.")

except serial.SerialException:
    print(f"\n[ERROR] Could not open port {SERIAL_PORT}.")
    print(
        "Make sure the Arduino IDE Serial Monitor is CLOSED and check your COM port."
    )
except KeyboardInterrupt:
    print("\nLogging stopped by user.")