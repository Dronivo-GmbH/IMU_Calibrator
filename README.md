# IMU_Calibrator

Desktop calibration tool for **BMI160** IMU boards (ESP32 / Arduino). Stream live sensor data, run calibration workflows, visualize orientation in 3D, and save calibration to JSON or flash it to the device.

**Developer:** [Apoorv Kulkarni](https://ak-apoorvkulkarni.github.io/)

---

## Features

- **Live visualization** — accelerometer, gyroscope, and magnitude plots in physical units (m/s², °/s)
- **3D body orientation** — IMU block tilts from gravity (accel-based tilt)
- **Calibration workflows** — gyro zero-rate, accel flat (+Z), six-face accel wizard, magnetometer figure-8
- **Stored calibration** — offsets and scales saved to `~/.config/dronivo/bmi160_calibration.json`
- **Runtime low-pass filter** — smooth accel + gyro display (not saved to JSON; separate from calibration)
- **Device sync** — `PING`, `GET_CAL`, `SET_CAL` over serial (SparkFun firmware)
- **Export / import** — JSON calibration profiles

---

## Hardware

| Setup | Notes |
|--------|--------|
| **DFRobot BMI160 + ESP32** | Streams `ax,ay,az,gx,gy,gz` CSV @ 115200 (raw LSB) |
| **SparkFun 9-DoF** | Optional `mx,my,mz`; use `firmware/bmi160_stream/` |

**Serial formats supported**

```text
ax,ay,az,gx,gy,gz              # 6-axis (DFRobot)
ax,ay,az,gx,gy,gz,mx,my,mz     # 9-axis
```

---

## Quick start (GUI)

```bash
cd imu_calibration_gui
python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python imu_calibration_gui.py
```

1. Select your **COM port** and click **Connect**
2. Open **Live Visualization** for charts and 3D orientation
3. Open **Calibration** to run workflows and view readouts

---

## Calibration pipeline

Calibration (saved) and filtering (runtime) are kept separate:

```text
BMI160 → Raw serial → LSB → m/s² / °/s → Apply JSON calibration → low-pass filter → Display
```

**Saved to JSON**

- `gyro_offset` — deg/s  
- `accel_offset` — m/s²  
- `accel_scale` — unitless  

**Not saved** — low-pass filter state (resets each session)

### Suggested order

1. **Gyro — Keep still (10s)** — device flat and motionless  
2. **Accel — Flat (+Z up, 10s)** or **Six-face wizard**  
3. **Magnetometer — Figure-8 (30s)** — only with 9-axis stream  
4. **Write to Device** — if firmware supports `SET_CAL`

---

## Firmware (optional)

Arduino sketch: `firmware/bmi160_stream/bmi160_stream.ino`

**Libraries:** SparkFun BMI160, ArduinoJson v6  

**Upload (Arduino CLI example)**

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/bmi160_stream
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 firmware/bmi160_stream
```

---

## Project layout

```text
IMU_Calibrator/
├── README.md
├── firmware/
│   └── bmi160_stream/          # ESP32 / Arduino streamer + SET_CAL
└── imu_calibration_gui/
    ├── imu_calibration_gui.py  # Main Tkinter app
    ├── calibration.py          # Cal math + JSON store
    ├── filtering.py            # Runtime low-pass (accel + gyro)
    ├── sensor_units.py         # LSB → physical units
    ├── imu_visualizer.py       # Live charts + 3D block
    ├── six_face_wizard.py      # PX4-style six-face accel wizard
    ├── serial_client.py        # Serial parse + capture
    └── requirements.txt
```

---

## Requirements

- Python 3.10+
- `pyserial`, `numpy`, `matplotlib`
- USB serial port to IMU board

---

## License

Use and modify for your projects. Attribution appreciated.
