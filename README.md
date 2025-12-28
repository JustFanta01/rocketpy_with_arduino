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


### Example of console output, with sampling_rate=20Hz (50ms):
```bash
t_sim = 0
t_sim = 1.0
t_sim = 2.0
t_sim = 3.0
t_sim = 4.0
t_sim = 5.0
[ESP32_cmd]: deployment_level=0.0
[ESP32_cmd]: deployment_level=0.009999999776482582
[ESP32_cmd]: deployment_level=0.019999999552965164
[ESP32_cmd]: deployment_level=0.029999999329447746
t_sim = 6.0
...
t_sim = 12.0
t_sim = 13.0
t_sim = 14.0
t_sim = 15.0
[ESP32_cmd]: 'drogue_deployed'
t_sim = 15.950000000000001
t_sim = 16.95
t_sim = 17.95
t_sim = 18.95
...
t_sim = 111.95
t_sim = 112.95
[ESP32_cmd] 'main_deployed'
t_sim = 113.9
t_sim = 114.9
...
t_sim = 152.9
t_sim = 153.9
Flight... COMPLETED
```
