"""
VL53L0X Full Data Extraction Script
====================================
Research: Assessing and Learning-Based Correction of ToF Distance Sensors
Student:  K.M.L. Lakmal Abeywickrama (20APP5026)

Extracts:
  - Distance (mm)
  - Range Status
  - Signal Rate (MCPS) via raw register
  - Ambient Count via raw register

Ground-truth distance is measured automatically by an HC-SR04 ultrasonic sensor.

Requirements:
  pip install adafruit-circuitpython-vl53l0x
  pip install smbus2
  pip install RPi.GPIO
"""

import time
import board
import busio
import adafruit_vl53l0x
import smbus2
import csv
import os
import statistics
import RPi.GPIO as GPIO
from datetime import datetime

# ─────────────────────────────────────────────
# HC-SR04 ULTRASONIC SENSOR (ground-truth)
# ─────────────────────────────────────────────
HCSR04_TRIG_PIN = 23   # GPIO pin connected to TRIG
HCSR04_ECHO_PIN = 24   # GPIO pin connected to ECHO
HCSR04_TIMEOUT  = 0.04 # 40 ms max echo wait (~6.8 m range)
SOUND_SPEED_MPS = 343.0

def setup_ultrasonic():
    """Initialise GPIO for HC-SR04."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(HCSR04_TRIG_PIN, GPIO.OUT)
    GPIO.setup(HCSR04_ECHO_PIN, GPIO.IN)
    GPIO.output(HCSR04_TRIG_PIN, False)
    time.sleep(0.05)  # let the sensor settle

def measure_ultrasonic_mm():
    """
    Take a single HC-SR04 distance measurement.
    Returns distance in mm, or None on timeout/error.
    """
    # Send 10 µs trigger pulse
    GPIO.output(HCSR04_TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(HCSR04_TRIG_PIN, False)

    deadline = time.monotonic() + HCSR04_TIMEOUT

    # Wait for echo to go HIGH
    while GPIO.input(HCSR04_ECHO_PIN) == 0:
        if time.monotonic() > deadline:
            return None
    pulse_start = time.monotonic()

    # Wait for echo to go LOW
    while GPIO.input(HCSR04_ECHO_PIN) == 1:
        if time.monotonic() > deadline:
            return None
    pulse_end = time.monotonic()

    duration_s = pulse_end - pulse_start
    distance_mm = (duration_s * SOUND_SPEED_MPS / 2.0) * 1000.0
    return round(distance_mm, 1)

def measure_ultrasonic_averaged(n=9):
    """
    Take n HC-SR04 measurements, drop outliers (top & bottom),
    and return the median in mm. Returns None if too many failures.
    """
    readings = []
    for _ in range(n):
        d = measure_ultrasonic_mm()
        if d is not None and 20 < d < 4000:  # sensor valid range: 2 cm – 4 m
            readings.append(d)
        time.sleep(0.06)  # HC-SR04 needs ≥ 60 ms between pings

    if len(readings) < 3:
        return None
    return round(statistics.median(readings), 1)

# ─────────────────────────────────────────────
# RAW REGISTER ADDRESSES (VL53L0X datasheet)
# ─────────────────────────────────────────────
REG_RESULT_RANGE_STATUS         = 0x14  # Range status byte
REG_RESULT_SIGNAL_RATE          = 0x1E  # Signal rate result (2 bytes, fixed-point)
REG_RESULT_AMBIENT_RATE         = 0x26  # Ambient rate count  (2 bytes)
REG_RESULT_RANGE_MM             = 0x1E  # Also used internally by Adafruit
REG_IDENTIFICATION_MODEL_ID     = 0xC0  # Should return 0xEE for VL53L0X

# I2C default address
VL53L0X_I2C_ADDR = 0x29

# ─────────────────────────────────────────────
# HELPER: Read 2-byte big-endian unsigned int
# ─────────────────────────────────────────────
def read_register_16(bus, register):
    """Read a 16-bit value from two consecutive registers."""
    try:
        data = bus.read_i2c_block_data(VL53L0X_I2C_ADDR, register, 2)
        return (data[0] << 8) | data[1]
    except Exception as e:
        return None

def read_register_8(bus, register):
    """Read a single byte register."""
    try:
        data = bus.read_i2c_block_data(VL53L0X_I2C_ADDR, register, 1)
        return data[0]
    except Exception as e:
        return None

# ─────────────────────────────────────────────
# RANGE STATUS DECODER
# ─────────────────────────────────────────────
RANGE_STATUS_DESCRIPTIONS = {
    0: "Valid",
    1: "Sigma Fail (low confidence)",
    2: "Signal Fail (low signal)",
    3: "Min Range Fail (too close)",
    4: "Phase Fail",
    5: "Hardware Fail",
}

def decode_range_status(raw_status):
    """Extract the 4-bit range status from the status register."""
    # Bits [7:3] are the range status, bit 0 is the 'data ready' flag
    status_code = (raw_status >> 3) & 0x1F
    # Remap to 0-5 scale (ST API mapping)
    if status_code == 11:
        return 0, "Valid"
    elif status_code == 7:
        return 1, "Sigma Fail"
    elif status_code == 2:
        return 2, "Signal Fail"
    elif status_code == 1:
        return 3, "Min Range Fail"
    elif status_code == 14:
        return 4, "Phase Fail"
    else:
        return 5, f"Other/Unknown (raw={status_code})"

# ─────────────────────────────────────────────
# SIGNAL RATE CONVERSION
# ─────────────────────────────────────────────
def raw_to_mcps(raw_value):
    """
    Convert raw fixed-point signal rate to MCPS.
    VL53L0X uses 9.7 fixed-point format (9 integer bits, 7 fractional bits).
    """
    if raw_value is None:
        return None
    return raw_value / 128.0  # divide by 2^7

# ─────────────────────────────────────────────
# SINGLE READING FUNCTION
# ─────────────────────────────────────────────
def get_full_reading(sensor, bus):
    """
    Perform one measurement and return all available data fields.
    Returns a dict with distance, status, signal_rate, ambient_count.
    """
    result = {
        "timestamp":    datetime.now().isoformat(),
        "distance_mm":  None,
        "range_status": None,
        "status_desc":  None,
        "signal_rate_mcps": None,
        "ambient_count":    None,
        "signal_raw":       None,
        "ambient_raw":      None,
    }

    try:
        # 1. Distance via Adafruit library (triggers a measurement)
        result["distance_mm"] = sensor.range

        # Small delay to let result registers settle
        time.sleep(0.005)

        # 2. Range status from register
        raw_status = read_register_8(bus, REG_RESULT_RANGE_STATUS)
        if raw_status is not None:
            status_code, status_desc = decode_range_status(raw_status)
            result["range_status"] = status_code
            result["status_desc"]  = status_desc

        # 3. Signal rate (MCPS) — register 0x1E, 2 bytes
        #    Note: these registers are populated AFTER a ranging cycle
        signal_raw = read_register_16(bus, 0x1E)
        result["signal_raw"]        = signal_raw
        result["signal_rate_mcps"]  = raw_to_mcps(signal_raw)

        # 4. Ambient count — register 0x26, 2 bytes
        ambient_raw = read_register_16(bus, 0x26)
        result["ambient_raw"]   = ambient_raw
        # Ambient is also in 9.7 fixed-point
        result["ambient_count"] = raw_to_mcps(ambient_raw) if ambient_raw is not None else None

    except Exception as e:
        result["error"] = str(e)

    return result

# ─────────────────────────────────────────────
# BURST SAMPLING (100 samples per point)
# ─────────────────────────────────────────────
def burst_sample(sensor, bus, n_samples=100, label="unknown", true_distance_mm=None):
    """
    Collect a burst of n_samples readings and compute statistics.
    Returns raw rows + summary statistics.
    """
    readings = []
    print(f"\n[BURST] Surface: {label} | True distance: {true_distance_mm}mm | Samples: {n_samples}")
    print("-" * 60)

    for i in range(n_samples):
        reading = get_full_reading(sensor, bus)
        reading["surface"]           = label
        reading["true_distance_mm"]  = true_distance_mm
        reading["sample_index"]      = i + 1
        readings.append(reading)

        # Progress indicator every 20 samples
        if (i + 1) % 20 == 0:
            print(f"  Sample {i+1}/{n_samples} — dist={reading['distance_mm']}mm | "
                  f"signal={reading['signal_rate_mcps']} MCPS | "
                  f"ambient={reading['ambient_count']} | "
                  f"status={reading['status_desc']}")
        time.sleep(0.033)  # ~30Hz measurement rate

    # Compute burst statistics
    distances   = [r["distance_mm"] for r in readings if r["distance_mm"] is not None]
    signals     = [r["signal_rate_mcps"] for r in readings if r["signal_rate_mcps"] is not None]
    ambients    = [r["ambient_count"] for r in readings if r["ambient_count"] is not None]
    dropouts    = sum(1 for r in readings if r["range_status"] != 0)

    stats = {
        "surface":            label,
        "true_distance_mm":   true_distance_mm,
        "n_samples":          n_samples,
        "mean_distance":      round(statistics.mean(distances), 2)   if distances else None,
        "std_distance":       round(statistics.stdev(distances), 2)  if len(distances) > 1 else 0,
        "min_distance":       min(distances)  if distances else None,
        "max_distance":       max(distances)  if distances else None,
        "mean_signal_mcps":   round(statistics.mean(signals), 4)     if signals else None,
        "std_signal_mcps":    round(statistics.stdev(signals), 4)    if len(signals) > 1 else 0,
        "mean_ambient":       round(statistics.mean(ambients), 4)    if ambients else None,
        "dropout_count":      dropouts,
        "dropout_rate_pct":   round((dropouts / n_samples) * 100, 1),
        "abs_error_mm":       round(abs(statistics.mean(distances) - true_distance_mm), 2)
                              if distances and true_distance_mm else None,
    }

    print(f"\n[SUMMARY] Mean={stats['mean_distance']}mm | "
          f"Std={stats['std_distance']}mm | "
          f"Dropout={stats['dropout_rate_pct']}% | "
          f"AbsError={stats['abs_error_mm']}mm")

    return readings, stats

# ─────────────────────────────────────────────
# CSV LOGGER
# ─────────────────────────────────────────────
RAW_CSV      = "tof_raw_readings.csv"
SUMMARY_CSV  = "tof_burst_summary.csv"

RAW_FIELDS = [
    "timestamp", "surface", "true_distance_mm", "sample_index",
    "distance_mm", "range_status", "status_desc",
    "signal_rate_mcps", "signal_raw",
    "ambient_count", "ambient_raw"
]

SUMMARY_FIELDS = [
    "surface", "true_distance_mm", "n_samples",
    "mean_distance", "std_distance", "min_distance", "max_distance",
    "mean_signal_mcps", "std_signal_mcps",
    "mean_ambient", "dropout_count", "dropout_rate_pct", "abs_error_mm"
]

def append_to_csv(filepath, fieldnames, rows):
    file_exists = os.path.isfile(filepath)
    with open(filepath, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        if not file_exists:
            writer.writeheader()
        if isinstance(rows, list):
            writer.writerows(rows)
        else:
            writer.writerow(rows)

# ─────────────────────────────────────────────
# REGISTER PROBE (run this first to verify)
# ─────────────────────────────────────────────
def probe_sensor(bus):
    """Verify sensor identity and probe key registers."""
    print("\n" + "="*60)
    print("SENSOR PROBE")
    print("="*60)

    model_id = read_register_8(bus, REG_IDENTIFICATION_MODEL_ID)
    print(f"Model ID Register (0xC0): 0x{model_id:02X} — Expected: 0xEE")

    revision = read_register_8(bus, 0xC1)
    print(f"Module Type  (0xC1):      0x{revision:02X}")

    mask_rev = read_register_8(bus, 0xC2)
    print(f"Mask Revision (0xC2):     0x{mask_rev:02X}")

    print("\nKey result registers (take a dummy reading first):")
    print(f"  Status  (0x14): {read_register_8(bus, 0x14)}")
    print(f"  Signal  (0x1E): {read_register_16(bus, 0x1E)}")
    print(f"  Ambient (0x26): {read_register_16(bus, 0x26)}")
    print("="*60 + "\n")

# ─────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────
def main():
    # --- Setup ultrasonic sensor GPIO ---
    setup_ultrasonic()

    # --- Setup I2C ---
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_vl53l0x.VL53L0X(i2c)
    bus = smbus2.SMBus(1)  # /dev/i2c-1 on Raspberry Pi

    # Optional: increase timing budget for better accuracy
    sensor.measurement_timing_budget = 200000

    # --- Probe sensor first ---
    _ = sensor.range
    time.sleep(0.1)
    probe_sensor(bus)

    # ── 1. SETUP ──────────────────────────────────────────────
    print("\n" + "="*60)
    print("DAILY LOGGING SETUP")
    print("="*60)

    surface = input("Enter the wall type (e.g., white_wall, glass): ")

    print("\nMeasuring ground-truth distance with HC-SR04 ultrasonic sensor...")
    true_dist = measure_ultrasonic_averaged(n=9)
    if true_dist is None:
        print("[WARNING] Ultrasonic sensor failed to get a valid reading.")
        try:
            true_dist = float(input("Enter actual distance manually in mm (fallback): "))
        except ValueError:
            true_dist = 1000.0
            print(f"Invalid input. Defaulting to {true_dist} mm.")
    else:
        print(f"Ultrasonic ground-truth distance: {true_dist} mm")

    print(f"\nConfiguration: Surface='{surface}', Distance={true_dist} mm")
    print("Starting logging protocol...")

    # ── 2. MAIN LOGGING LOOP ─────────────────────────────────
    try:
        while True:
            now = datetime.now()

            if now.hour >= 24:
                print(f"\n[{now.strftime('%H:%M:%S')}] End of logging window. Done!")
                break

            if now.hour < 0:
                print(f"[{now.strftime('%H:%M:%S')}] Waiting for logging window...", end="\r")
                time.sleep(60)
                continue

            # Re-measure ground-truth distance before each burst
            print(f"\n[{now.strftime('%H:%M:%S')}] Re-measuring ground-truth distance...")
            updated_dist = measure_ultrasonic_averaged(n=9)
            if updated_dist is not None:
                true_dist = updated_dist
                print(f"  Updated ground-truth: {true_dist} mm")
            else:
                print(f"  Ultrasonic read failed — keeping last value: {true_dist} mm")

            print(f"[{now.strftime('%H:%M:%S')}] Starting data collection burst...")

            raw_readings, summary = burst_sample(
                sensor, bus,
                n_samples=100,
                label=surface,
                true_distance_mm=true_dist
            )

            append_to_csv(RAW_CSV, RAW_FIELDS, raw_readings)
            append_to_csv(SUMMARY_CSV, SUMMARY_FIELDS, summary)

            print(f"[{datetime.now().strftime('%H:%M:%S')}] Data saved to CSVs.")
            print("Sleeping for 5 minutes...\n")
            time.sleep(300)

    finally:
        bus.close()
        GPIO.cleanup()
        print("\n✓ System gracefully shut down.")

if __name__ == "__main__":
    main()