# rocketpy_with_arduino
Minimal Hardware-in-the-Loop (HIL) setup where an **Arduino** (will be an ESP32) interacts with a **RocketPy simulation** over USB serial using a **custom binary protocol**.

The Arduino acts as the external flight controller:
- pulls simulation data at a fixed rate,
- consumes all produced samples (no overwrite),
- sends immediate command frames (parachutes, air-brakes).

The Python side runs RocketPy and exposes a custom controller that serves telemetry and reacts to commands.

---

## Requirements

- Arduino
- USB cable
- Python ≥ 3.10

---

## Setup

### 1. Flash ESP32 Firmware
Flash the provided Arduino firmware i.e. ```arduino.ino```.

### 2. Clone RocketPy
```bash
git clone https://github.com/RocketPy-Team/RocketPy.git
cd RocketPy
```

### 3. Python Environment
```bash
python3 -m venv venv
source venv/bin/activate
```

### 4. Modify RocketPy
- add ```rocket_v2.py``` in ```RocketPy/rocketpy/rocket/```,
- modify the ```__init__.py``` in ```RocketPy/rocketpy``` and ```RocketPy/rocket/rocket``` to expose ```RocketV2``` class,
- change ```RocketPy/rocketpy/simulation/flight.py#L3746```, comment ```# tmp_dict[time]._controllers += node._controllers```.

### 5. Install RocketPy from source
```bash
pip install -r requirements.txt
pip install .
```

### 6. Run the simulation
- plug in the Arduino
- ```bash
  python3 Reanalysis_v10.py
  ```
