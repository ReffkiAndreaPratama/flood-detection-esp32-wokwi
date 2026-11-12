# FloodGuard X — Smart Flood Intelligence System

An advanced ESP32/Wokwi simulation for multi-point flood monitoring, risk scoring, trend estimation, a web command dashboard, and a servo actuator demonstration.

> **Safety notice:** Educational prototype only. It is not a certified flood warning system and must not be used for real evacuation or life-safety decisions.

## What it does

- Two HC-SR04 sensors simulate upstream and downstream water-surface measurements.
- Water level is estimated as `300 cm reference height - measured distance`.
- Two potentiometers simulate rainfall intensity and flow proxy (0–100%).
- Risk engine combines maximum water level, rain, flow proxy, and positive water-level trend into a 0–100 risk score.
- Four states: NORMAL, WASPADA, SIAGA, BAHAYA.
- Trend rate in cm/min and estimated time to 220 cm critical level when the measured trend is positive.
- OLED telemetry, RGB status LED, buzzer patterns, physical MUTE and AUTO/MANUAL buttons.
- Servo models a floodgate actuator. In AUTO mode it moves to 90° when risk is BAHAYA; in MANUAL mode it follows dashboard commands.
- Responsive web dashboard with live telemetry, event log, threshold controls, and REST-like JSON API.

## Hardware / Wokwi parts

- ESP32 DevKit C V4
- 2 × HC-SR04 ultrasonic sensors
- 2 × potentiometers (rain and flow proxy)
- SSD1306 I2C OLED (128×64)
- Servo
- Buzzer
- Common-cathode RGB LED + 3 × 220 Ω resistors
- 2 × pushbuttons

## Pin map

| Component | Signal | ESP32 GPIO |
|---|---|---:|
| Upstream HC-SR04 | TRIG / ECHO | 5 / 34 |
| Downstream HC-SR04 | TRIG / ECHO | 18 / 35 |
| Rain potentiometer | SIG | 32 |
| Flow potentiometer | SIG | 33 |
| OLED SSD1306 | SDA / SCL | 21 / 22 |
| Servo floodgate | PWM | 13 |
| Buzzer | signal | 19 |
| RGB LED | R / G / B | 25 / 26 / 27 |
| MUTE button | signal | 23 |
| MODE button | signal | 4 |

Power:
- HC-SR04 VCC → 5V; GND → GND.
- Potentiometers and OLED → 3V3/GND.
- Servo V+ → 5V; GND → common GND.
- RGB LED common cathode → GND; each color leg is connected through its own 220 Ω resistor.
- Buttons connect GPIO to GND; firmware uses `INPUT_PULLUP`.

## Wokwi setup

1. Extract this ZIP.
2. Open https://wokwi.com and create a new ESP32 project.
3. Replace `diagram.json` and `sketch.ino` with the files in this folder.
4. Add the following libraries in the Wokwi Library Manager if prompted:
   - `Adafruit SSD1306`
   - `Adafruit GFX Library`
   - `ESP32Servo`
5. Start simulation and open Serial Monitor at 115200 baud.

The Wokwi editor does not provide a universal ZIP-import action in every workflow. Creating a new ESP32 project and replacing the two project files is the reliable method.

## Simulating a flood

Each HC-SR04 has a distance slider/attribute. The sensor is treated as mounted 300 cm above the reference waterline:
- Increase distance → lower estimated water level.
- Decrease distance → higher estimated water level.
- Move the RAIN potentiometer to increase rain intensity.
- Move the FLOW potentiometer to change the flow proxy.

Try these scenarios:
1. Normal: distance 220 cm or more; low rain.
2. Watch: reduce one distance until estimated water exceeds 100 cm.
3. Alert: reduce distance until estimated water exceeds 160 cm.
4. Danger: reduce distance until estimated water exceeds 220 cm; the automatic servo response is activated.

## Dashboard

Firmware serves the dashboard at `/` and status JSON at `/api/status`, on HTTP port 80.

For local access use Wokwi Private IoT Gateway with `wokwi.toml` port forwarding:
- Local: `localhost:8180`
- Target: `target:80`

Then open `http://localhost:8180`. Dashboard access from the browser may not work in a plain Wokwi web simulation without the Private IoT Gateway/port forwarding. Sensor readings, OLED, RGB LED, buzzer, and servo simulation can still operate.

## API endpoints

- `GET /` — dashboard
- `GET /api/status` — current telemetry, risk score, trend, actuator and event history
- `GET /api/control?mode=auto`
- `GET /api/control?mode=manual`
- `GET /api/control?gate=open` — switches to MANUAL and opens the servo
- `GET /api/control?gate=close` — switches to MANUAL and closes the servo
- `GET /api/control?mute=toggle`
- `GET /api/control?alarm=test`
- `GET /api/thresholds?warning=100&alert=160&danger=220`

## PlatformIO

`platformio.ini` and `src/main.cpp` are included. Open the extracted folder in VS Code with PlatformIO installed. The source in `src/main.cpp` is kept identical to `sketch.ino`.

Dependencies:
- Adafruit GFX Library
- Adafruit SSD1306
- ESP32Servo

## Engineering limitations / physical deployment notes

- HC-SR04 ECHO is 5 V logic. Use a resistor divider or level shifter before connecting ECHO to a physical ESP32 GPIO.
- Standard HC-SR04 is not waterproof; use an outdoor-rated waterproof ultrasonic sensor for field deployment.
- Servo should use a suitable separate 5 V supply in real hardware; connect grounds together. Do not power a loaded servo from the ESP32 3V3 pin.
- Potentiometers are simulation stand-ins, not real rain gauges or flow meters.
- The `flowProxy` input is a user-adjustable demonstration parameter, not a measured physical flow velocity.
- Trend/ETA are simple short-window linear estimates; they are not hydrological forecasts.
- Floodgate servo behavior is illustrative. Real gates require engineered actuators, mechanical interlocks, fail-safe design, and professional review.

## Validation performed for this package

- JSON syntax parsed successfully.
- Every connection endpoint references a declared component.
- Firmware GPIO definitions checked against the diagram connection map.
- `sketch.ino` and `src/main.cpp` byte-for-byte identical.
- ZIP integrity test passed.

A live Wokwi simulation and ESP32 compiler build were not run in this environment. Verify in Wokwi/PlatformIO before physical deployment.
