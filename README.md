<p align="center"><img src="logo/navcore_logo.svg" alt="NavCore" width="460"></p>

NavCore is a two-wheeled autonomous robot built on an ESP32. It drives a straight route from point A to point B with no line on the floor, no camera and no external positioning. It estimates its own position from wheel encoders and a gyroscope, detects obstacles with three laser time-of-flight sensors, measures each obstacle while driving around it and returns to the route. The robot serves its own web application over Wi-Fi, so a phone or laptop shows the live map, the path driven, the sensor readings and every decision the robot takes.

## Video demonstration

[Watch the NavCore demonstration](https://arianadaboc.github.io/NavCore/demo.html)

The video covers four scenarios: one box on the route, two boxes in a row, a box placed while the robot is turning, and a custom route with a box placed near the destination.

## Project files

- [`latest-version/`](latest-version/) — the current, improved code running on the robot: ESP32 firmware and the web application
- [`backup-version/`](backup-version/) — the older working version, kept as a backup
- [`docs/NavCore_Technical_Documentation.pdf`](docs/NavCore_Technical_Documentation.pdf) — technical documentation with formulas, diagrams and test results
- [`demo/`](demo/) — demonstration video
- [`logo/`](logo/) — project logo

## Hardware

| Component | Role | Connection |
| --- | --- | --- |
| ESP32-WROOM-32 | control loop (core 1) and web server (core 0) | — |
| TB6612FNG dual H-bridge | motor direction and speed, PWM 20 kHz, active braking | right AIN1 27 · AIN2 14 · PWMA 13, left BIN1 25 · BIN2 26 · PWMB 4 |
| 2 DC gear motors + 2 encoders | differential drive, distance from encoder pulses | encoders on GPIO 32 / 33 |
| MPU6050 | heading from the Z-axis gyroscope (±500 °/s) | I²C, SDA 21 · SCL 22, address 0x68 |
| 3 × VL53L0X | front, left and right distance, 20 ms per reading | I²C, XSHUT 23 / 19 / 18, addresses 0x31 / 0x30 / 0x32 |
| Status LEDs | green = way clear, red = obstacle or avoidance in progress | red GPIO 15 · green GPIO 12, 220–330 Ω each |
| Power | 4 × AA (≈ 6 V) for the motors, LM2596S buck converter for the 5 V rail, 470 µF capacitors on the driver and ESP32 supply | common ground |

Robot size: 24 × 16 cm, sensors 13.5 cm in front of the wheel axle.

## How the robot works

**Localisation.** The robot keeps its position in a frame attached to the route: x along A→B, y sideways from the line, θ the angle to the route. Every new piece of distance from the encoders is split with the gyroscope heading:

```
ds = Δ((nL + nR) / 2) / 1.8768 pulses per cm
x += ds · cos θ        y += ds · sin θ
```

The gyroscope offset is measured at every start, and the distance and rotation scales are calibrated from the app (a 200 cm drive and a 360° turn) and stored on the robot.

**Driving.** A proportional heading controller keeps the robot straight (base PWM 130, about 21 cm/s). It aims at a point on the line 35 cm ahead, so any sideways offset is removed gradually.

**Turning.** Turns on the spot run in two stages: a fast turn that brakes early by a learned overshoot estimate, then short correction pulses until the heading is within ±2°.

**Obstacle avoidance.**

| Step | What the robot does |
| --- | --- |
| Detect | 3 front readings below 270 mm in a row; closer than 220 mm it backs up first |
| Choose side | averages the left and right distances and takes the wider side (at least 250 mm free) |
| Arc out | turns away on an arc of up to 60° and finds the obstacle edge with the front sensor |
| Pass | drives parallel at edge + 12 cm, while the side sensor tracks the obstacle until its rear |
| Return | arcs back to the line, aligns to ±2° and continues to B |

An object closer than 10 cm during a manoeuvre makes the robot back up and start a new avoidance. An object within 30 cm of B is treated as the destination. If neither side has room, the robot stops safely.

## Web application

The application is a single HTML page stored in the firmware and served by the robot.

- live map with the route, placed boxes, driven path, sensor cones and measured obstacles (unexpected ones in pink)
- status cards: state, position, angle, the three distances, progress, avoidances, Wi-Fi signal
- event log with every decision and its values, and export of a complete run as JSON
- map editor and saved custom scenarios
- distance and rotation calibration
- offline simulator that runs the same avoidance logic without the robot

The page polls the robot every 100 ms over HTTP:

| Request | Purpose |
| --- | --- |
| `GET /` | the application |
| `GET /api/state?since=E&ps=P` | state, position, distances, new events and new trail points |
| `GET /api/cmd?c=start&route=L` | start a run of L cm (100–600) |
| `GET /api/cmd?c=stop` / `c=reset` | stop the run / set the position back to A |
| `GET /api/cmd?c=calib&ppcm=…&gs=…` | store new calibration constants |

## Running the project

1. Install the Arduino IDE with the ESP32 board package and the **VL53L0X** library by Pololu.
2. Copy `latest-version/firmware/navcore_wifi/wifi_secrets.example.h` to `wifi_secrets.h` and fill in the Wi-Fi name and password (2.4 GHz).
3. Open `latest-version/firmware/navcore_wifi/navcore_wifi.ino`, select the ESP32 Dev Module board and upload.
4. Open the Serial Monitor at 115200 baud to read the robot's address and open it in a browser on the same network. If the robot cannot join the network within 30 s, it creates its own access point `NavCore` at `192.168.4.1`.
5. Place the robot at A facing B, set the route length and press **Start**.

To try the application without the robot, open `latest-version/app/navcore_app.html` in a browser. The **Start** button then runs the simulator.
