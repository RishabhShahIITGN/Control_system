# Ball/Car-and-Beam Control System

**ES 245 — Control Systems Laboratory | Course Project Part-II**

A complete MATLAB + Arduino implementation of system identification, frequency-domain analysis, compensator design, and hardware control for a sliding-car-on-beam balancing system.

---

## Repository Layout

```text
Control_system/
├── data/
│   └── task5_chirp_runs.csv           # Experimental chirp input-output recordings
├── matlab/
│   ├── task5_system_identification.m  # Task 5: Plant identification from chirp data
│   ├── task6_analysis.m               # Task 6: Root Locus, Nyquist, Bode analysis
│   ├── task7_compensator_design.m     # Task 7: Lead compensator design & verification
│   ├── task5_preprocessing.png        # Figure: Chirp input & sensor response
│   ├── task5_model_validation.png     # Figure: Measured vs. simulated plant response
│   ├── task5_bode_plot.png            # Figure: Open-loop Bode diagram
│   ├── task6_root_locus.png           # Figure: Root Locus with design boundaries
│   ├── task6_nyquist.png              # Figure: Nyquist diagram (global + zoomed)
│   ├── task6_bode.png                 # Figure: Open-loop Bode with stability margins
│   ├── task6_design_criteria_eval.png # Figure: Uncompensated step response vs. specs
│   ├── task7_bode_comparison.png      # Figure: Bode comparison (G vs. CG)
│   ├── task7_step_response_verification.png  # Figure: Compensated step response
│   └── task7_root_locus_compensated.png      # Figure: Compensated Root Locus
├── sketches/
│   ├── Chirp_Signal_Ultrasonic.ino    # Open-loop chirp excitation for data collection
│   ├── P_controller/                  # Part-1: Proportional controller (IR sensor)
│   ├── PD_controller/                 # Part-1: PD controller (IR sensor)
│   ├── PID_controller/                # Part-1: PID controller (IR sensor)
│   └── Task8_Lead_Controller/         # Task 8: Lead compensator (ultrasonic sensor)
│       └── Task8_Lead_Controller.ino
├── docs/
│   ├── Controls_Report.pdf
│   └── ES 245 Course Project Part 2.pdf
├── LICENSE
└── README.md
```

---

## Hardware Setup

| Component | Arduino Pin | Notes |
| :--- | :---: | :--- |
| HC-SR04 Ultrasonic Trigger | D9 | 5V digital output |
| HC-SR04 Ultrasonic Echo | A1 | 5V digital input (used as digital) |
| Servo Signal | D3 | Standard hobby servo (PWM) |
| Servo Power | External 5V–6V | **Do not power from Arduino 5V pin** |

The beam is attached to the servo horn via a mechanical linkage. The ultrasonic sensor is mounted at one end of the beam and measures the distance to the sliding car.

> **Physical Convention:** Increasing servo angle tilts the beam such that the car accelerates *toward* the sensor (distance decreases). This is reflected in the negative plant gain $K = -0.6463$.

---

## Identified Plant Model

All three MATLAB scripts use the same validated plant transfer function derived in Task 5:

$$G(s) = \frac{-0.6463}{s\(s + 0.0170)} \quad \left[\frac{\text{cm}}{\text{deg}}\right]$$

| Parameter | Symbol | Value | Unit |
| :--- | :---: | :---: | :--- |
| Plant Gain | $K$ | $-0.6463$ | cm/s² per deg |
| Track Friction | $b$ | $0.0170$ | s⁻¹ |
| Servo Neutral Trim | $u_0$ | $107.59$ | degrees |
| Model Fit | — | $79.94\%$ | NRMSE |
| RMSE | — | $2.31$ | cm |

The plant is a **double integrator with light viscous friction** — essentially a free-rolling car on a tilted beam with negligible resistance.

---

## Task 5: System Characterization & Identification

**Script:** [`matlab/task5_system_identification.m`](matlab/task5_system_identification.m) — *271 lines, fully self-contained*  
**Marks:** 15  
**Data Source:** `amp25_run2` from [`data/task5_chirp_runs.csv`](data/task5_chirp_runs.csv)

### What This Script Does

1. **Loads and preprocesses** the experimental chirp data:
   - Reads the CSV file and isolates the `amp25_run2` experiment (±25° servo swing, 0.1–5.0 Hz chirp over 20 s).
   - Resamples onto a uniform 50 Hz grid, removes ultrasonic dropouts via linear interpolation, and applies a 7-point median + Gaussian filter to suppress sensor noise.
   - Automatically **truncates** the record at the point where the car hits the physical bumper stop (~12.4 s), since data beyond this point is nonlinear.

2. **Derives the continuous-time transfer function** $G(s) = K / [s(s+b)]$:
   - Uses `fminsearch` to optimize five parameters simultaneously: friction $b$, gain $K$, servo trim $u_0$, initial position $y_0$, and initial velocity $v_0$.
   - The cost function minimizes the sum of squared errors between the measured output and a **ZOH-exact discrete simulation** (helper function `simPlant`), which integrates the state-space equations $\ddot{y} + b\dot{y} = K(u - u_0)$ using matrix exponentials.

3. **Cross-validates** the result against MATLAB's `n4sid` subspace identification (2nd-order model) for independent confirmation.

4. **Discretizes** to $G(z)$ via zero-order hold at $T_s = 0.02$ s and prints the equivalent Arduino C++ difference equation.

5. **Generates 3 figures:**

| Figure | File | Content |
| :--- | :--- | :--- |
| **Figure 1** | `task5_preprocessing.png` | Top: Chirp servo input (raw + smoothed) with neutral trim line. Bottom: Ultrasonic distance response with bumper truncation marker. |
| **Figure 2** | `task5_model_validation.png` | Top: Measured vs. simulated plant response overlay (79.94% fit). Bottom: Residual error with ±2σ bounds. |
| **Figure 3** | `task5_bode_plot.png` | Bode magnitude and phase of the identified plant with Gain Margin and Phase Margin annotations. |

### Key Code Logic

```
CSV data → resample 50 Hz → median + Gaussian filter → truncate at bumper
     ↓
fminsearch optimization loop:
  for each candidate [b, K, u0, y0, v0]:
    simulate ZOH plant:  x1[k+1] = x1[k] + a12*x2[k] + B1*uk
                         x2[k+1] = e^(-b*Ts)*x2[k] + B2*uk
    compute sum-of-squares error vs. measured y
     ↓
optimal parameters → tf(K, [1 b 0]) → G(s) → c2d(G, Ts, 'zoh') → G(z)
```

### How the `simPlant` Helper Works

The function at the bottom of the script performs exact ZOH (zero-order hold) discrete simulation of:

$$\ddot{y} + b\dot{y} = K \cdot (u - u_0)$$

It converts this to a first-order state-space system with states $[y, \dot{y}]$ and computes the exact discrete-time state transition matrices using the analytical matrix exponential of the continuous $A$ matrix. This avoids numerical integration errors that would accumulate over the 600+ sample identification window.

---

## Task 6: Root Locus and Frequency Response Analysis

**Script:** [`matlab/task6_analysis.m`](matlab/task6_analysis.m) — *227 lines*  
**Marks:** 15  
**Input:** Hardcoded plant parameters from Task 5 ($K = 0.6463$, $b = 0.0170$)

### What This Script Does

The script has three sections matching the three sub-tasks:

#### Section 6.1 — Root Locus & Nyquist Analysis

- **Root Locus** (`rlocus(G)`): Plots the trajectory of closed-loop poles as proportional gain $K_p$ varies from 0 to ∞.
  - Overlays the **damping ratio boundary** ($\zeta \ge 0.707$, drawn by `sgrid`) — closed-loop poles must be inside this wedge for OS < 5%.
  - Overlays the **settling time boundary** ($\sigma \ge 0.6$ s⁻¹, drawn by `xline(-0.6)`) — closed-loop poles must be to the left of this line for $T_s < 5$ s.
  - Marks the **breakaway point** at $\sigma_b = -b/2 = -0.0085$ where the two real root locus branches meet and become complex conjugates.
  - **Key observation:** The root locus branches travel almost vertically upward from the breakaway point. The real part of the dominant poles is locked at $-0.0085$ regardless of gain — this is why proportional control alone cannot meet the design specs.

- **Nyquist Diagram**: Two subplots:
  1. **Global view** (`nyquist(G)`): Shows the full Nyquist contour with the critical point $(-1, j0)$ marked.
  2. **Zoomed view** near $(-1, j0)$: Shows that the Nyquist curve passes within a distance of 0.021 from the critical point, confirming the dangerously low phase margin. The gain crossover point is marked on the contour.

#### Section 6.2 — Bode Plot & Stability Margins

- Calls `margin(G)` to compute and plot:
  - **Gain Margin** $G_m = \infty$ dB (phase never reaches $-180°$, so the system is gain-stable for all $K_p > 0$).
  - **Phase Margin** $P_m = 1.21°$ at $\omega_{cp} = 0.8038$ rad/s (nearly zero — system is on the verge of oscillatory instability).
- Explains the frequency-domain slope behavior:
  - $-20$ dB/decade at low frequencies (single integrator).
  - $-40$ dB/decade at high frequencies (double-pole roll-off beyond $\omega = b$).

#### Section 6.3 — Design Criteria Evaluation

- Simulates the **closed-loop step response** for three proportional gains $K_p = \{0.5, 1.0, 2.0\}$ using `feedback(Kp * G, 1)`.
- For each gain, computes:
  - Natural frequency: $\omega_n = \sqrt{K_p \cdot K}$
  - Damping ratio: $\zeta = b / (2\omega_n)$ — always $\ll 0.707$ (severely underdamped).
  - Overshoot: ~96.7% (target: < 5%) — **FAILS**.
  - 5% Settling time: ~353 s (target: < 5 s) — **FAILS**.
- Draws a green "target spec box" on the step response plot (the region where $0.95 \le y \le 1.05$ and $t \le 5$ s). None of the P-control responses enter this box.
- **Conclusion:** Proportional control is fundamentally inadequate. A **Lead Compensator** (phase-lead / filtered PD) is strictly required.

#### Section 6 — Correlation Summary

Prints a formatted table correlating stability margins across all three representations (Root Locus, Nyquist, Bode), showing they all tell the same story: the system is gain-stable but phase-critical.

### Figures Generated

| Figure | File | Content |
| :--- | :--- | :--- |
| **Root Locus** | `task6_root_locus.png` | Root Locus with $\zeta$ and $\sigma$ design boundaries, breakaway point |
| **Nyquist** | `task6_nyquist.png` | Global + zoomed Nyquist with proximity to $(-1, j0)$ |
| **Bode** | `task6_bode.png` | Open-loop Bode diagram with $G_m$ and $P_m$ annotations |
| **Step Response** | `task6_design_criteria_eval.png` | Closed-loop response for 3 gains vs. target spec box |

---

## Task 7: Compensator Identification — Lead or Lag Design

**Script:** [`matlab/task7_compensator_design.m`](matlab/task7_compensator_design.m) — *311 lines*  
**Marks:** 10  
**Input:** Same plant parameters as Task 6

### Why Lead, Not Lag?

| Criterion | Lag Compensator | Lead Compensator |
| :--- | :--- | :--- |
| **Phase contribution** | Adds negative phase ($\Delta\phi < 0$) | Adds positive phase ($\Delta\phi > 0$) |
| **Effect on $P_m = 1.21°$** | Pushes phase below $-180°$ → **unstable** | Boosts phase above $-180°$ → **stable** |
| **Effect on bandwidth** | Reduces bandwidth → slower response | Maintains or increases bandwidth |
| **Effect on settling time** | Worse (already 353 s) | Dramatically better (< 3 s) |
| **Verdict** | **REJECTED** | **REQUIRED** |

### Designed Lead Compensator

$$C(s) = 4.50 \cdot \frac{s + 0.050}{s + 3.50}$$

| Parameter | Value | Purpose |
| :--- | :---: | :--- |
| Gain $K_c$ | $4.50$ | Sets the closed-loop bandwidth and crossover gain |
| Zero $z_c$ | $0.050$ rad/s | Placed near the plant friction pole to extend low-frequency phase lead |
| Pole $p_c$ | $3.50$ rad/s | High-frequency roll-off — limits sensor noise amplification |
| Lead ratio $\alpha = z_c / p_c$ | $0.0143$ | Gives a theoretical maximum phase lead of $76.4°$ |

### What This Script Does

1. **Defines the compensator** `C(s)` and forms the compensated open-loop `L(s) = series(C, G)`:
   - Uses `series(C, G)` (not `C * G`) to correctly route the signal flow: Error → C(s) → Servo → G(s) → Distance.

2. **Frequency-domain verification** (Section 4):
   - Computes margins of both $G(s)$ and $L(s) = C(s)G(s)$ using `margin()`.
   - Plots a **comparative Bode diagram** with both curves on the same axes.
   - Shows the phase margin boost: $1.21° \to 74.63°$ ($+73.42°$).
   - Annotates the crossover frequencies and the phase margin improvement.

3. **Time-domain verification** (Section 5):
   - Computes closed-loop step responses for both uncompensated ($K_p = 1$) and compensated systems using `feedback()`.
   - Uses `stepinfo(..., 'SettlingTimeThreshold', 0.05)` for the **5% settling time** (matching the project specification exactly — MATLAB defaults to 2%).
   - Plots both step responses with the green target specification box and marks the peak overshoot point.

4. **Root Locus of compensated system** (Section 6):
   - Plots `rlocus(L)` showing how the compensator zero reshapes the root locus.
   - Overlays the same $\zeta \ge 0.707$ and $\sigma \ge 0.6$ design boundaries from Task 6.
   - Marks the **operating closed-loop poles** (from `pole(T_comp)`) with yellow pentagrams, confirming they lie inside the acceptable region.

5. **Discrete-time compensator** (Section 7):
   - Converts $C(s)$ to $C(z)$ using `c2d(C, Ts, 'tustin')` (bilinear/Tustin transform at $T_s = 0.02$ s).
   - Extracts the numerator and denominator coefficients and prints the exact difference equation:

   ```
   u[k] = 0.932367 * u[k-1] + 4.350000 * e[k] - 4.345652 * e[k-1]
   servo_deg = 107.59 + u
   ```

### Design Results Summary

| Metric | Target | Uncompensated | Compensated | Status |
| :--- | :---: | :---: | :---: | :---: |
| Phase Margin | > 60° | 1.21° | **74.63°** | ✅ |
| Overshoot | < 5% | 96.7% | **3.28%** | ✅ |
| Settling Time (5%) | < 5 s | 352.9 s | **2.56 s** | ✅ |
| Steady-State Error | 0 | 0.00 cm | **0.00 cm** | ✅ |
| Gain Margin | Stable | ∞ dB | **∞ dB** | ✅ |

### Figures Generated

| Figure | File | Content |
| :--- | :--- | :--- |
| **Bode Comparison** | `task7_bode_comparison.png` | Magnitude & phase of $G(s)$ vs. $L(s) = C(s)G(s)$ |
| **Step Response** | `task7_step_response_verification.png` | Uncompensated vs. compensated closed-loop step response |
| **Root Locus** | `task7_root_locus_compensated.png` | Compensated root locus with operating poles marked |

---

## Task 8: Hardware Implementation (Arduino)

**Sketch:** [`sketches/Task8_Lead_Controller/Task8_Lead_Controller.ino`](sketches/Task8_Lead_Controller/Task8_Lead_Controller.ino)  
**Marks:** 15

This sketch implements the exact discrete Lead Compensator from Task 7 on Arduino hardware using the HC-SR04 ultrasonic sensor. See the sketch header for full documentation, wiring, and the serial tuning interface.

### Quick Start

1. Wire the HC-SR04 and servo as shown in the Hardware table above.
2. Open `Task8_Lead_Controller.ino` in Arduino IDE. Compile and upload.
3. Open Serial Monitor at **115200** baud.
4. The servo will hold at trim ($107.6°$) for 3 seconds, then auto-enable control.
5. Use the serial commands (`G`, `T`, `R`, `D`, `S`, `?`) to tune on-the-fly.

---

## Signal Flow Diagram

The complete feedback control system implemented across Tasks 5–8:

```
                            Task 7 Lead Compensator
                         ┌──────────────────────────┐
   Setpoint    ┌───┐     │        s + 0.050          │     ┌─────────────────────┐    Distance
   r(t) ──────►│ + ├──►  │ C(s) = 4.50 · ─────────── │ ──► │       -0.6463       │ ──► y(t)
     (cm)      │   │     │        s + 3.500           │     │ G(s) = ──────────── │    (cm)
               └─┬─┘     └──────────────────────────┘     │      s(s + 0.017)   │
                 │ -                                       └─────────────────────┘
                 │                                                   │
                 └───────────────────────────────────────────────────┘
                            HC-SR04 Ultrasonic Sensor
```

**Discrete-time (Arduino at 50 Hz):**

$$u[k] = 0.932367\,u[k\!-\!1] + 4.350000\,e[k] - 4.345652\,e[k\!-\!1]$$

$$\theta_{\text{servo}}[k] = 107.6° - u[k]$$

(The sign flip `−u` compensates for the negative plant gain $K = -0.6463$.)

---

## How to Run the MATLAB Scripts

### Prerequisites
- MATLAB R2019b or later (or MATLAB Online)
- Control System Toolbox (`tf`, `margin`, `rlocus`, `feedback`, `c2d`)
- System Identification Toolbox (optional — only for `n4sid` cross-check in Task 5)

### Execution Order

The scripts are independent and can be run in any order, but the logical flow is:

```
Task 5 (identify plant) → Task 6 (analyze plant) → Task 7 (design compensator)
```

1. **Task 5:** Open `matlab/task5_system_identification.m` and press **Run** (`F5`).
   - Requires `data/task5_chirp_runs.csv` (auto-detected via relative paths).
   - Generates 3 PNG figures in `matlab/` and prints the identified transfer function.

2. **Task 6:** Open `matlab/task6_analysis.m` and press **Run** (`F5`).
   - Uses hardcoded parameters from Task 5 (no data file needed).
   - Generates 4 PNG figures in `matlab/`.

3. **Task 7:** Open `matlab/task7_compensator_design.m` and press **Run** (`F5`).
   - Uses same hardcoded parameters.
   - Generates 3 PNG figures in `matlab/` and prints the discrete C(z) coefficients.

All scripts support section-by-section execution (`Ctrl+Enter` on each `%%` block).

---

## Legacy Part-1 Sketches

The `P_controller/`, `PD_controller/`, and `PID_controller/` folders contain the Part-1 implementations using an analog Sharp IR distance sensor on pin A1. These use a different sensor, different trim angle (99°), and different units (meters). They are retained for reference but are **not used** in Part-2 Tasks 5–8.

| Sketch | Gains | Setpoint | Sensor | Baud |
| :--- | :--- | :---: | :--- | :---: |
| `P_controller` | $K_p = 7.0$ | 30 cm | IR (Sharp GP2Y0A21) | 19200 |
| `PD_controller` | $K_p = 3.0$, $K_d = 2.45$ | 10 cm | IR (Sharp GP2Y0A21) | 115200 |
| `PID_controller` | $K_p = 3.0$, $K_d = 2.45$, $K_i = 0.40$ | 10 cm | IR (Sharp GP2Y0A21) | 115200 |

---

## Project Documents

| File | Description |
| :--- | :--- |
| `docs/Controls_Report.pdf` | Part-1 project report |
| `docs/ES 245 Course Project Part 2.pdf` | Part-2 task sheet and requirements |
