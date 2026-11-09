# Ball-and-Beam Control System

Arduino implementations of proportional (P), proportional-derivative (PD), and proportional-integral-derivative (PID) controllers for a ball-and-beam system. Project reports are in `docs/`.

## Repository Layout

```text
Control_system/
|-- docs/
|   |-- Controls_Report.pdf
|   `-- Microsoft Word - ES245-course-project - part-2 (1).docx - ES 245 Course Project Part 2.pdf
|-- sketches/
|   |-- P_controller/
|   |   `-- P_controller.ino
|   |-- PD_controller/
|   |   `-- PD_controller.ino
|   `-- PID_controller/
|       `-- PID_controller.ino
|-- LICENSE
`-- README.md
```

Each controller is kept in its own Arduino sketch folder. Open the matching folder or `.ino` file in the Arduino IDE.

## Hardware

The sketches expect the following connections and components:

| Component | Arduino connection |
| --- | --- |
| IR distance sensor analog output | A1 |
| Beam servo signal | Digital pin 3 |

The distance conversion in the sketches is calibrated for a Sharp GP2Y0A21YK0F sensor. Using a different sensor may require changing `readDistanceMeters()` and recalibrating the measurements. The servo's horizontal reference is configured as 99 degrees, with a maximum beam tilt of 35 degrees in either direction; adjust these constants for the actual mechanism before operation.

## Controller Configurations

| Sketch | Controller gains | Setpoint | Serial baud |
| --- | --- | --- | --- |
| `P_controller` | Kp = 7.0 | 0.30 m (30 cm) | 19200 |
| `PD_controller` | Kp = 3.0, Kd = 2.45 | 0.10 m (10 cm) | 115200 |
| `PID_controller` | Kp = 3.0, Kd = 2.45, Ki = 0.40 | 0.10 m (10 cm) | 115200 |

The P sketch's setpoint constant is `0.30 m`; its adjacent source comment still says `0.10 m`. The table reflects the constant currently used by the code.

## Build and Run

1. Connect the sensor output to A1 and the servo signal to digital pin 3. Ensure the sensor, servo, and Arduino share a common ground, and use an appropriate power supply for the servo.
2. Open one of the sketch folders in the Arduino IDE and select the correct board and port.
3. Compile and upload the sketch.
4. Open Serial Monitor or Serial Plotter at the baud rate listed above. The sketches output `Setpoint_cm`, `Position_cm`, and `Servo_deg` at approximately 50 Hz.

Test with the mechanism secured and clear of obstructions. Tune the gains, setpoint, servo direction, and mechanical limits for your hardware before running the system freely.

## Project Documents

- `docs/Controls_Report.pdf`
- `docs/Microsoft Word - ES245-course-project - part-2 (1).docx - ES 245 Course Project Part 2.pdf`
