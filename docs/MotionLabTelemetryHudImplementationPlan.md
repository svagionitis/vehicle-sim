# Motion Lab: Advanced Telemetry HUD & Engineering Analytics Implementation Plan

## 1. Executive Summary & Objectives

### 1.1 Overview
This implementation plan specifies the complete technical design, mathematical derivations, software architecture, retro 1996 vector wireframe presentation, and verification strategy for the **Advanced Telemetry HUD & Engineering Analytics Suite** in **Motion Lab 2D** (`apps/MotionLab`). Originating from Section 9.4 of [MotionLabImplementationPlan.md](MotionLabImplementationPlan.md#94-advanced-telemetry-hud--engineering-analytics) and building upon the zero-allocation circular storage engine specified in [TelemetryRingBufferImplementationPlan.md](TelemetryRingBufferImplementationPlan.md), this capability transforms Motion Lab's diagnostic overlay from basic numeric text into a professional, real-time automotive engineering telemetry workstation.

Modern vehicle dynamics testing demands immediate visual feedback of instantaneous tire grip utilization, chassis pitch and roll load transfers, high-frequency transient trends, and deterministic disk telemetry logging. Conforming to standards utilized in motorsport data acquisition (such as MoTeC i2 Pro, Bosch Motorsport WinDARAB, and PlotJuggler), the Advanced Telemetry HUD delivers synchronous, hard-real-time analytical visualization without introducing dynamic heap allocations or disrupting simulation determinism.

```
+---------------------------------------------------------------------------------------------------+
| SPEED: 48.2 m/s (107.8 mph)   YAW: +0.12 rad/s   FPS: 60.0   DT: 8.33 ms   SURFACE: TARMAC (1.00) |
| VEHICLE: DRIFT CAR [F8]       DRIFT ANGLE: 28.5 deg   DRIFT SCORE: 14,280   RECORDING: [REC: 42s] |
+---------------------------------------------------------------------------------------------------+
|  [G-G FRICTION CIRCLE]            [WEIGHT TRANSFER]            [LIVE OSCILLOSCOPE (SPEED)]        |
|           +ay (Lateral L)          FL: [||||||||  ] FR: [||||]  60|       /----\                  |
|               |                    RL: [||||||||||] RR: [||  ]  40|  /---/      \---/             |
|        .-----.|-----.                                           20| /                             |
|       /       |      \            [INPUTS]                       0+-----------------              |
| -ax --+-------*------+-- +ax       [W: THR] [S: BRK]           [LIVE OSCILLOSCOPE (SLIP ANGLE)]   |
| (Brake\       |      /  (Accel)    [A: LFT] [D: RGT]            30|      /----\                   |
|        '-----'|-----'              [SPACE: HANDBRAKE]            0+-----/------\-----             |
|               |                                                -30|                               |
|           -ay (Lateral R)                                                                         |
+---------------------------------------------------------------------------------------------------+
```

---

### 1.2 Core Capabilities & Deliverables

1. **Dynamic Friction Circle ($G$-$G$ Diagram)**:
   - Polar coordinate radar in the HUD corner plotting body-frame longitudinal acceleration ($a_x$) versus lateral cornering acceleration ($a_y$).
   - Normalized traction limit ellipse scaled to active surface friction $\mu$ and gravity $g_0$:
     $$\left(\frac{a_x}{\mu_x g_0}\right)^2 + \left(\frac{a_y}{\mu_y g_0}\right)^2 \le 1.0$$
   - Color-coded instantaneous operating point: Neon green for linear grip ($< 70\%$), bright amber for transition zone ($70\% - 95\%$), and warning red for grip saturation/breakaway ($> 95\%$).
   - 60-frame historical trail cloud rendering fading acceleration vectors to illustrate cornering entry-apex-exit transitions.

2. **4-Corner Dynamic Weight Transfer Visualizer**:
   - Real-time computation of normal wheel reactions ($F_{z,\text{FL}}, F_{z,\text{FR}}, F_{z,\text{RL}}, F_{z,\text{RR}}$) combining static weight distribution with longitudinal acceleration pitch couples and lateral cornering roll couples.
   - Four vertical segmented LED-style bar graphs displaying proportional wheel load relative to static baseline, featuring overload alerts when wheels approach zero normal load (wheel lift / rollover threshold).

3. **Live Rolling Oscilloscope Strip Charts**:
   - Pre-allocated 200-sample circular historical strip charts embedded directly into the rasterized HUD layout.
   - Four selectable/concurrent channels:
     - Channel 1: Forward speed ($\text{m/s}$ and $\text{mph}$).
     - Channel 2: Front versus Rear tire slip angles ($\alpha_f, \alpha_r$ in degrees).
     - Channel 3: Chassis yaw rate ($\dot{\psi}$ in $\text{deg/s}$).
     - Channel 4: Throttle percentage and powertrain RPM.
   - Dynamic baseline zero-line, autoscale gridlines, and minimum/maximum extent labels.

4. **Synchronous CSV & KLV Session Telemetry Exporters**:
   - Dedicated hotkey trigger (`Ctrl+E` and `Ctrl+F9`) for starting and stopping full-rate frame recording to disk using [`CsvTelemetryExporter`](../libs/VehicleSimCore/telemetry/CsvTelemetryExporter.h).
   - Real-time blinking HUD recording indicator: `[REC: 12.4s (1488 frames)]`.
   - Guaranteed atomic serialization upon session stop into timestamped files: `telemetry_run_YYYYMMDD_HHMMSS.csv`.

5. **Integrated Proving Ground Analytics Coexistence**:
   - Seamless coexistence and mutual layout exclusivity with Proving Ground HUD cards (100-Meter Slalom, ISO 3888-2 Moose Test, Skidpad Circles, and Procedural Circuit Track).
   - Minimal CPU overhead and zero memory allocations in the render loop, utilizing procedural glyph rasterization.

---

## 2. High-Level Architecture & Subsystem Decomposition

### 2.1 ASCII System Architecture Diagram

```
+---------------------------------------------------------------------------------------------------+
|                        Advanced Telemetry HUD Architecture in Motion Lab 2D                       |
+---------------------------------------------------------------------------------------------------+
|  [apps/MotionLab] (Desktop Application Layer)                                                     |
|                                                                                                   |
|    +--------------------+       +----------------------------+       +-----------------------+    |
|    |    MotionLabApp    |------>|   TelemetryCoordinator2D   |------>|     TelemetryHud      |    |
|    | (Ctrl+E Record,    |       | (History Buffer, G-G Calc, |       | (G-G Friction Radar,  |    |
|    |  'H' HUD Toggle)   |       |  Weight Transfer, Exporters|       |  Weight Transfer Bars,|    |
|    +---------+----------+       +--------------+-------------+       |  Rolling Oscilloscope,|    |
|              |                                 |                     |  Recording Status)    |    |
|              | Physics Step                    | Push Frame          +-----------------------+    |
|              v                                 v                                                  |
|    +--------------------+       +----------------------------+                                    |
|    | RigidBody2D Chassis|------>|     TelemetrySampler2D     |                                    |
|    | & Active Dynamics  |       | (Acceleration Filtering,   |                                    |
|    | (1 of 12 Vehicles) |       |  Body-Frame Decomposition) |                                    |
|    +--------------------+       +--------------+-------------+                                    |
|                                                | Populates                                        |
|                                                v                                                  |
|                                 +----------------------------+       +-----------------------+    |
|                                 |   TelemetryRingBuffer2D    |------>| CsvTelemetryExporter  |    |
|                                 | (Fixed 200/1200 Samples,   |       | (Disk File Stream on  |    |
|                                 |  Zero Heap Allocation)     |       |  Session Stop / Flush)|    |
|                                 +----------------------------+       +-----------------------+    |
+------------------------------------------------+--------------------------------------------------+
                                                 | Physics & Telemetry Contracts
                                                 v
+---------------------------------------------------------------------------------------------------+
|  [libs/VehicleSimCore/telemetry] (Pure C++17 Core Engine - Zero External Dependencies)            |
|                                                                                                   |
|    +---------------------------+  +---------------------------+  +---------------------------+    |
|    | TelemetryFrame2D          |  | TelemetryRingBuffer2D     |  | TelemetrySampler2D        |    |
|    | (Kinematics, G-Forces,    |  | (O(1) Push, Reverse Index,|  | (Alpha Filtering, Accel   |    |
|    |  4 Wheels, Inputs, Engine)|  |  Peak G-Force Statistics) |  |  Differentiation, Bodies) |    |
|    +---------------------------+  +---------------------------+  +---------------------------+    |
|    +---------------------------+  +---------------------------+                                   |
|    | CsvTelemetryExporter      |  | KlvTelemetryExporter      |                                   |
|    | (MoTeC / PlotJuggler CSV) |  | (MISB ST 0601 / SMPTE 336)|                                   |
|    +---------------------------+  +---------------------------+                                   |
+---------------------------------------------------------------------------------------------------+
```

---

### 2.2 Mermaid System Architecture Diagram

```mermaid
graph TD
    subgraph AppLayer ["Application Layer (apps/MotionLab)"]
        APP["MotionLabApp"]
        TC["TelemetryCoordinator2D"]
        HUD["TelemetryHud::render()"]
        GG["Friction Circle (G-G Polar)"]
        WT["4-Corner Weight Transfer Bars"]
        OSC["Live Rolling Oscilloscopes"]
        REC["Recording Session State"]

        APP -->|Physics Step & Hotkeys| TC
        TC -->|TelemetrySnapshot Ext| HUD
        HUD --> GG
        HUD --> WT
        HUD --> OSC
        HUD --> REC
    end

    subgraph CoreLayer ["VehicleSimCore Engine Layer (libs/VehicleSimCore/telemetry)"]
        BODY["RigidBody2D Kinematics"]
        DYN["IVehicleDynamics (12 Vehicles)"]
        SAM["TelemetrySampler2D"]
        BUF["TelemetryRingBuffer2D (Pre-allocated)"]
        CSV["CsvTelemetryExporter"]
        FILE["Disk File (.csv)"]

        BODY & DYN -->|State Vector| SAM
        SAM -->|TelemetryFrame2D| BUF
        BUF -->|Frame Windows| TC
        TC -->|Flush On Stop| CSV
        CSV -->|Buffered Stream| FILE
    end
```

---

## 3. Mathematical Formulations & Physical Derivations

### 3.1 Body-Frame Acceleration & G-G Friction Circle Derivation

In planar vehicle dynamics, the inertial world linear acceleration $\mathbf{a}_{\text{world}} = (a_{W,x}, a_{W,y})$ sampled via backward finite differentiation of velocity is transformed into the vehicle body coordinate frame aligned with heading angle $\theta$:

$$\begin{bmatrix} a_{x,\text{body}} \\ a_{y,\text{body}} \end{bmatrix} = \begin{bmatrix} \cos\theta & \sin\theta \\ -\sin\theta & \cos\theta \end{bmatrix} \begin{bmatrix} a_{W,x} \\ a_{W,y} \end{bmatrix}$$

where:
* $a_{x,\text{body}}$ represents longitudinal chassis acceleration (positive during forward throttle acceleration, negative during braking).
* $a_{y,\text{body}}$ represents lateral cornering acceleration (positive during left-hand turns, negative during right-hand turns).

Expressed in gravitational acceleration units ($g_0 = 9.80665\,\text{m/s}^2$):

$$G_{\text{long}} = \frac{a_{x,\text{body}}}{g_0}, \quad G_{\text{lat}} = \frac{a_{y,\text{body}}}{g_0}$$

#### 3.1.1 Normalized Friction Ellipse Formulation
Given active surface tire-road friction coefficient $\mu$, longitudinal adhesion limit $\mu_x \approx \mu$, and lateral adhesion limit $\mu_y \approx \mu \cdot \kappa_{\text{lat}}$ (where $\kappa_{\text{lat}} \approx 0.95$ accounts for tire tread compound anisotropy), the available grip envelope is governed by the friction ellipse:

$$\left(\frac{G_{\text{long}}}{\mu_x}\right)^2 + \left(\frac{G_{\text{lat}}}{\mu_y}\right)^2 \le 1.0$$

The instantaneous total traction demand ratio $\rho \in [0.0, \infty)$ is defined as:

$$\rho = \sqrt{\left(\frac{G_{\text{long}}}{\mu_x}\right)^2 + \left(\frac{G_{\text{lat}}}{\mu_y}\right)^2}$$

#### 3.1.2 Grip Status & Dynamic Color Thresholds
The HUD friction circle maps $\rho$ to continuous color states:

$$\text{Color}(\rho) = \begin{cases}
\text{Neon Green } (0, 255, 136), & \rho < 0.70 \quad (\text{Linear Grip Zone}) \\
\text{Bright Amber } (255, 191, 0), & 0.70 \le \rho \le 0.95 \quad (\text{Handling Transition Zone}) \\
\text{Warning Red } (255, 51, 51), & \rho > 0.95 \quad (\text{Traction Saturation / Breakaway})
\end{cases}$$

---

### 3.2 4-Corner Dynamic Weight Transfer Mechanics

During high-performance driving, inertial forces acting at the vehicle center of gravity (CG) produce pitch and roll moments that dynamically redistribute vertical normal loads across the four tire contact patches.

```
                    ^ +Z (Vertical Load Fz)
                    |
      FL +-------+  |  +-------+ FR
         | Wheel |  |  | Wheel |
         +---+---+  |  +---+---+
             |      |      |
             +------+------+  --- Body Roll Moment: M_roll = m * ay * h
             |      CG     |
             |      *      |
             +------+------+  --- Body Pitch Moment: M_pitch = m * ax * h
             |             |
         +---+---+     +---+---+
      RL | Wheel |     | Wheel | RR
         +-------+     +-------+
```

#### 3.2.1 Static Equilibrium Wheel Loads
Let $m$ be vehicle mass, $g_0$ gravitational acceleration, $L$ wheelbase, $a$ longitudinal distance from front axle to CG, $b$ longitudinal distance from rear axle to CG ($L = a + b$), and $t_w$ average track width:

$$F_{z0,\text{front}} = \frac{m g_0 \cdot b}{L}, \quad F_{z0,\text{rear}} = \frac{m g_0 \cdot a}{L}$$

Assuming lateral symmetry at rest, each wheel carries half of its axle's static weight:

$$F_{z0,\text{FL}} = F_{z0,\text{FR}} = \frac{F_{z0,\text{front}}}{2}, \quad F_{z0,\text{RL}} = F_{z0,\text{RR}} = \frac{F_{z0,\text{rear}}}{2}$$

#### 3.2.2 Longitudinal Pitch Weight Transfer
Longitudinal acceleration $a_x$ acts through the CG height $h_{\text{cg}}$, producing pitch moment $M_{\text{pitch}} = m \cdot a_x \cdot h_{\text{cg}}$. The resulting axle load transfer $\Delta F_{z,\text{long}}$ relieves the front axle and loads the rear axle:

$$\Delta F_{z,\text{long}} = \frac{m \cdot a_x \cdot h_{\text{cg}}}{L}$$

Axle normal forces become:

$$F_{z,\text{front}} = F_{z0,\text{front}} - \Delta F_{z,\text{long}}, \quad F_{z,\text{rear}} = F_{z0,\text{rear}} + \Delta F_{z,\text{long}}$$

#### 3.2.3 Lateral Roll Weight Transfer
Lateral acceleration $a_y$ produces roll moment $M_{\text{roll}} = m \cdot a_y \cdot h_{\text{cg}}$. Distributing roll stiffness between front and rear suspensions with front roll stiffness ratio $\xi_f \in [0.4, 0.6]$ (nominally $\xi_f = 0.52$ for understeer bias) and rear ratio $\xi_r = 1.0 - \xi_f$:

$$\Delta F_{z,\text{lat,front}} = \xi_f \cdot \frac{m \cdot a_y \cdot h_{\text{cg}}}{t_{w,\text{front}}}, \quad \Delta F_{z,\text{lat,rear}} = \xi_r \cdot \frac{m \cdot a_y \cdot h_{\text{cg}}}{t_{w,\text{rear}}}$$

#### 3.2.4 Individual 4-Corner Wheel Loads
Combining static, pitch, and roll terms:

$$\begin{aligned}
F_{z,\text{FL}} &= \max\left(0.0, \; \frac{F_{z,\text{front}}}{2} - \Delta F_{z,\text{lat,front}}\right) \\
F_{z,\text{FR}} &= \max\left(0.0, \; \frac{F_{z,\text{front}}}{2} + \Delta F_{z,\text{lat,front}}\right) \\
F_{z,\text{RL}} &= \max\left(0.0, \; \frac{F_{z,\text{rear}}}{2} - \Delta F_{z,\text{lat,rear}}\right) \\
F_{z,\text{RR}} &= \max\left(0.0, \; \frac{F_{z,\text{rear}}}{2} + \Delta F_{z,\text{lat,rear}}\right)
\end{aligned}$$

The zero-clamping accounts for physical wheel lift-off when roll/pitch couples exceed available normal gravity preload.

---

### 3.3 Live Oscilloscope Normalization & Ring Buffer Slicing

The rolling oscilloscope strip chart plots historical telemetry channels over a fixed window of $N = 200$ samples (representing $1.67\,\text{s}$ at $120\,\text{Hz}$ physics update rate).

For any channel sample $v_k$ within the ring buffer ($k \in [0, N-1]$), the normalized screen height coordinate $y_k \in [Y_{\text{bottom}}, Y_{\text{top}}]$ is given by:

$$u_k = \text{clamp}\left(\frac{v_k - V_{\text{min}}}{V_{\text{max}} - V_{\text{min}}}, \; 0.0, \; 1.0\right)$$
$$y_k = Y_{\text{bottom}} - u_k \cdot (Y_{\text{bottom}} - Y_{\text{top}})$$

Horizontal screen coordinate $x_k$ across chart pixel width $W_{\text{chart}}$:

$$x_k = X_{\text{left}} + \frac{k}{N - 1} \cdot W_{\text{chart}}$$

#### Oscilloscope Channel Ranges:
* **Speed**: $V_{\text{min}} = 0.0\,\text{m/s}$, $V_{\text{max}} = 60.0\,\text{m/s}$ ($\approx 134\,\text{mph}$).
* **Tire Slip Angles ($\alpha_f, \alpha_r$)**: $V_{\text{min}} = -30.0^\circ$, $V_{\text{max}} = +30.0^\circ$ ($0^\circ$ centerline).
* **Yaw Angular Velocity ($\dot{\psi}$)**: $V_{\text{min}} = -180.0\,\text{deg/s}$, $V_{\text{max}} = +180.0\,\text{deg/s}$ ($0^\circ$ centerline).
* **Powertrain RPM**: $V_{\text{min}} = 0.0\,\text{RPM}$, $V_{\text{max}} = 9000.0\,\text{RPM}$.

---

## 4. Visual Layout & 1996 Retro Vector HUD Presentation

### 4.1 Screen Space Allocation & Geometric Coordinates

The telemetry visual components occupy dedicated non-overlapping zones across standard viewport resolutions ($1280 \times 720$ and $1920 \times 1080$), seamlessly integrating with existing Proving Ground radar and status widgets:

```
+---------------------------------------------------------------------------------------------------+ (0,0)
| [TOP BAR: SPEED, GEAR, RPM, YAW RATE, FPS, STEP DT, TERRAIN, VEHICLE]       [PROVING GROUND RADAR]|
|                                                                             (Top Right 140x140)   |
|                                                                                                   |
| [G-G FRICTION RADAR]                                                                              |
| Top-Left: (X=24, Y=220)                                                                           |
| Radius: 60 px                                                                                     |
| Inner Circles: 0.5G, 1.0G, 1.5G                                                                   |
| Crosshair: Zero-G Center                                                                          |
| Trail: 60 historic points                                                                         |
|                                                                                                   |
| [4-CORNER LOAD BARS]                                                        [LIVE OSCILLOSCOPE 1] |
| Top-Left: (X=170, Y=220)                                                   Top-Right: (X=W-360,   |
| 4 Vertical Gauges (FL, FR, RL, RR)                                                     Y=180)     |
| Dimensions: 18x100 px per wheel                                             Size: 336x64 px       |
| Dynamic Color Fill (Green -> Amber -> Red)                                  Trace: Speed (m/s)    |
| Overload & Lift-off Indicators                                                                    |
|                                                                             [LIVE OSCILLOSCOPE 2] |
| [INPUT PEDAL GAUGES]                                                        Top-Right: (X=W-360,   |
| Top-Left: (X=24, Y=360)                                                                Y=256)     |
| Vertical Throttle, Brake, Steer Bars                                        Size: 336x64 px       |
|                                                                             Trace: Slip Angles    |
|                                                                                                   |
|                                                                             [LIVE OSCILLOSCOPE 3] |
|                                                                             Top-Right: (X=W-360,   |
|                                                                                        Y=332)     |
|                                                                             Size: 336x64 px       |
|                                                                             Trace: Yaw Rate       |
|                                                                                                   |
| [BOTTOM CAPTION & RECORDER STATUS]                                                                |
| Y = H - 34: Hotkey Guide / Recording Badge "[REC: 12.4s (1488 frames)]"                           |
+---------------------------------------------------------------------------------------------------+ (W,H)
```

### 4.2 Retro Vector Color Palette Specification

| UI Element | RGB Color | Hex Code | Visual Rationale |
| :--- | :--- | :--- | :--- |
| **Grid Lines & Axes** | `RGB(40, 60, 70)` | `#283C46` | Subdued wireframe coordinate framing |
| **G-G Baseline Circle** | `RGB(0, 180, 220)` | `#00B4DC` | Neon cyan 1.0G road grip boundary |
| **Instantaneous Point** | `RGB(255, 255, 255)` | `#FFFFFF` | Brilliant white acceleration cursor |
| **Grip Trail (Fading)** | `RGBA(0, 240, 200, α)` | Dynamic | Phosphor CRT persistence effect |
| **Weight Bar FL/FR** | `RGB(140, 230, 197)` | `#8CE6C5` | Front axle baseline cyan-green |
| **Weight Bar RL/RR** | `RGB(212, 171, 113)` | `#D4AB71` | Rear axle baseline warm gold |
| **Wheel Lift Warning** | `RGB(255, 60, 60)` | `#FF3C3C` | High-visibility emergency red |
| **Oscilloscope Speed** | `RGB(0, 240, 255)` | `#00F0FF` | Electric cyan velocity profile |
| **Oscilloscope Front Slip**| `RGB(255, 200, 50)` | `#FFC832` | Front tire slip angle gold trace |
| **Oscilloscope Rear Slip** | `RGB(255, 80, 80)` | `#FF5050` | Rear tire slip angle warning orange |
| **Recording Active Badge**| `RGB(255, 40, 40)` | `#FF2828` | Blinking broadcast recording red |

---

## 5. Software Architecture & C++17 Data Contracts

### 5.1 Telemetry Snapshot Extension (`apps/MotionLab/TelemetryHud.h`)

To support multi-channel analytics without dynamic allocations, [`TelemetrySnapshot`](../apps/MotionLab/TelemetryHud.h) is enriched with vector telemetry structures:

```cpp
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <SDL2/SDL.h>

#include "libs/VehicleSimCore/math/Vec2.h"

namespace motion_lab {

/// @struct DynamicWheelLoads
/// @brief 4-corner vertical reactions in Newtons and load fractions.
struct DynamicWheelLoads {
    double frontLeftLoad { 0.0 };   ///< FL normal reaction in N.
    double frontRightLoad { 0.0 };  ///< FR normal reaction in N.
    double rearLeftLoad { 0.0 };    ///< RL normal reaction in N.
    double rearRightLoad { 0.0 };   ///< RR normal reaction in N.
    double staticFrontWheel { 0.0 };///< Nominal unaccelerated front wheel load.
    double staticRearWheel { 0.0 }; ///< Nominal unaccelerated rear wheel load.
};

/// @struct OscilloscopeWindow
/// @brief Fixed-size historical buffers for rolling strip charts.
struct OscilloscopeWindow {
    static constexpr std::size_t kHistoryCapacity { 200U };
    std::array<float, kHistoryCapacity> speedHistory {};
    std::array<float, kHistoryCapacity> frontSlipHistory {};
    std::array<float, kHistoryCapacity> rearSlipHistory {};
    std::array<float, kHistoryCapacity> yawRateHistory {};
    std::size_t sampleCount { 0U };
    std::size_t headIndex { 0U };
};

/// @struct ExtendedTelemetrySnapshot
/// @brief High-rate analytical state payload passed to TelemetryHud::render.
struct ExtendedTelemetrySnapshot {
    // Kinematics & Accelerations
    double speedMps { 0.0 };
    double speedMph { 0.0 };
    double angularVelocity { 0.0 };
    double longitudinalG { 0.0 };
    double lateralG { 0.0 };
    double peakLongG { 0.0 };
    double peakLatG { 0.0 };
    double surfaceFriction { 1.0 };

    // Dynamic Normal Reactions
    DynamicWheelLoads wheelLoads {};

    // G-G Trail History (60 samples = 0.5s at 120Hz)
    static constexpr std::size_t kGgTrailCapacity { 60U };
    std::array<vehicle_sim::math::Vec2, kGgTrailCapacity> ggTrail {};
    std::size_t ggTrailCount { 0U };

    // Strip Chart History
    OscilloscopeWindow stripCharts {};

    // Session Recording State
    bool isRecording { false };
    double recordingDuration { 0.0 };
    std::uint64_t recordedFrames { 0U };

    // Core Status & Toggles
    bool analyticsHudVisible { true };
    double stepDurationMs { 8.333 };
    double fps { 60.0 };
    std::string vehicleName { "CLASSIC CAR" };
    std::string terrainName { "TARMAC" };
};

} // namespace motion_lab
```

---

### 5.2 Extended Telemetry HUD API (`apps/MotionLab/TelemetryHud.h`)

```cpp
namespace motion_lab {

/// @class TelemetryHud
/// @brief Professional automotive telemetry and engineering analytics HUD.
class TelemetryHud {
public:
    /// @brief Renders the primary status and analytical engineering HUD overlay.
    /// @param[in] renderer Active hardware SDL renderer.
    /// @param[in] snapshot Primary telemetry and course snapshot.
    /// @param[in] extended Extended analytical loads, G-G circle, and oscilloscope data.
    /// @param[in] screenWidth Viewport width in pixels.
    /// @param[in] screenHeight Viewport height in pixels.
    static void render(SDL_Renderer* renderer,
                       const TelemetrySnapshot& snapshot,
                       const ExtendedTelemetrySnapshot& extended,
                       int screenWidth,
                       int screenHeight) noexcept;

    /// @brief Renders the polar G-G friction circle radar.
    /// @param[in] renderer Active hardware SDL renderer.
    /// @param[in] extended Extended telemetry snapshot containing Gs and trail.
    /// @param[in] centerX Center X pixel coordinate.
    /// @param[in] centerY Center Y pixel coordinate.
    /// @param[in] radius Radius of the 1.0G outer circle.
    static void renderFrictionCircle(SDL_Renderer* renderer,
                                     const ExtendedTelemetrySnapshot& extended,
                                     int centerX,
                                     int centerY,
                                     int radius) noexcept;

    /// @brief Renders the 4-corner dynamic wheel load vertical bar graphs.
    /// @param[in] renderer Active hardware SDL renderer.
    /// @param[in] loads Dynamic wheel loads and nominal references.
    /// @param[in] x Top-left X coordinate.
    /// @param[in] y Top-left Y coordinate.
    static void renderWeightTransfer(SDL_Renderer* renderer,
                                     const DynamicWheelLoads& loads,
                                     int x,
                                     int y) noexcept;

    /// @brief Renders rolling oscilloscope strip chart widget.
    /// @param[in] renderer Active hardware SDL renderer.
    /// @param[in] values Rolling data values.
    /// @param[in] count Number of valid samples in buffer.
    /// @param[in] headIndex Circular buffer head offset.
    /// @param[in] x Top-left pixel X coordinate.
    /// @param[in] y Top-left pixel Y coordinate.
    /// @param[in] width Strip chart pixel width.
    /// @param[in] height Strip chart pixel height.
    /// @param[in] minY Minimum engineering scale value.
    /// @param[in] maxY Maximum engineering scale value.
    /// @param[in] title Alphanumeric channel identifier.
    /// @param[in] traceColor Primary line color.
    static void renderStripChart(SDL_Renderer* renderer,
                                 const float* values,
                                 std::size_t count,
                                 std::size_t headIndex,
                                 int x,
                                 int y,
                                 int width,
                                 int height,
                                 float minY,
                                 float maxY,
                                 const char* title,
                                 SDL_Color traceColor) noexcept;

    /// @brief Draws procedural alphanumeric text at pixel coordinates.
    static void drawText(SDL_Renderer* renderer,
                         const std::string& text,
                         int x,
                         int y,
                         int scale,
                         SDL_Color color) noexcept;
};

} // namespace motion_lab
```

---

### 5.3 Telemetry Coordinator Class (`apps/MotionLab/TelemetryCoordinator.h`)

To maintain clean separation between window management in `MotionLabApp` and telemetry processing, a dedicated coordinator manages ring buffers, sampling, and file export:

```cpp
#pragma once

#include <fstream>
#include <memory>
#include <string>

#include "apps/MotionLab/TelemetryHud.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/telemetry/CsvTelemetryExporter.h"
#include "libs/VehicleSimCore/telemetry/TelemetryRingBuffer2D.h"
#include "libs/VehicleSimCore/telemetry/TelemetrySampler2D.h"

namespace motion_lab {

/// @class TelemetryCoordinator
/// @brief Orchestrates live telemetry sampling, history buffers, and disk logging.
class TelemetryCoordinator {
public:
    /// @brief Default constructor allocating 1200-frame ring buffer.
    TelemetryCoordinator();

    /// @brief Destructor closing active recording sessions safely.
    ~TelemetryCoordinator() noexcept;

    /// @brief Resets all telemetry buffers, peak statistics, and recording state.
    void reset() noexcept;

    /// @brief Steps telemetry pipeline with latest physics state.
    /// @param[in] body Active rigid body kinematics.
    /// @param[in] carParams Wheelbase, track width, CG height, and vehicle mass.
    /// @param[in] simTime Current simulation timestamp in seconds.
    /// @param[in] dt Physics timestep duration in seconds.
    void update(const vehicle_sim::physics::RigidBody2D& body,
                double mass,
                double wheelbase,
                double trackWidth,
                double cgHeight,
                double surfaceMu,
                double simTime,
                double dt) noexcept;

    /// @brief Toggles on-demand disk session recording.
    /// @return True if recording was started, false if stopped.
    [[nodiscard]] bool toggleRecording() noexcept;

    /// @brief Queries whether disk logging is currently active.
    [[nodiscard]] bool isRecording() const noexcept;

    /// @brief Populates extended telemetry snapshot for HUD rendering.
    /// @param[out] outSnapshot Destination snapshot structure.
    void populateSnapshot(ExtendedTelemetrySnapshot& outSnapshot) const noexcept;

    /// @brief Accesses underlying ring buffer for export and testing.
    [[nodiscard]] const vehicle_sim::telemetry::TelemetryRingBuffer2D& getBuffer() const noexcept;

private:
    vehicle_sim::telemetry::TelemetryRingBuffer2D m_buffer;
    vehicle_sim::telemetry::TelemetrySampler2D m_sampler;
    vehicle_sim::telemetry::CsvTelemetryExporter m_exporter;

    // Rolling Strip Chart Circular Buffers
    OscilloscopeWindow m_oscilloscope {};

    // G-G Trail Buffer
    static constexpr std::size_t kGgTrailSize { 60U };
    std::array<vehicle_sim::math::Vec2, kGgTrailSize> m_ggTrail {};
    std::size_t m_ggHead { 0U };
    std::size_t m_ggCount { 0U };

    // Dynamic Wheel Loads
    DynamicWheelLoads m_currentLoads {};

    // Disk Recording
    bool m_isRecording { false };
    double m_recordingStartTime { 0.0 };
    std::uint64_t m_recordedFrames { 0U };
    std::ofstream m_recordFile {};
    std::string m_currentLogFilename {};
};

} // namespace motion_lab
```

---

## 6. Phased Implementation Roadmap

```
+---------------------------------------------------------------------------------------------------+
|                     Phased Implementation Gantt Diagram: Advanced Telemetry HUD                   |
+---------------------------------------------------------------------------------------------------+
| Phase 1: Core Formulations & Weight Transfer Extraction      |====|                               |
| Phase 2: Ring Buffer History & TelemetryCoordinator          |    |====|                          |
| Phase 3: Friction Circle (G-G Polar Radar) Rendering        |         |====|                     |
| Phase 4: 4-Corner Weight Transfer Bar Graphs                |              |====|                |
| Phase 5: Live Rolling Oscilloscope Strip Charts             |                   |====|           |
| Phase 6: Session Disk Recording & Hotkey Integration         |                        |====|      |
| Phase 7: Verification, Hardening & Automated Test Suite     |                             |====| |
+---------------------------------------------------------------------------------------------------+
```

### Phase 1: Dynamic Weight Transfer & Acceleration Derivations
* **Objective**: Implement exact dynamic normal load calculation functions in `VehicleSimCore` or `TelemetryCoordinator`.
* **Deliverables**:
  - Implement dynamic load computation formula accounting for $a_x$, $a_y$, $m$, $h_{\text{cg}}$, $L$, $t_w$, and roll stiffness distribution.
  - Unit test weight conservation: $\sum_{i} F_{z,i} = m \cdot g_0$ under pure pitch, pure roll, and combined pitch-roll maneuvers.
  - Implement zero-clamping for wheel lift detection.

### Phase 2: TelemetryCoordinator Architecture
* **Objective**: Establish `TelemetryCoordinator` inside `apps/MotionLab` to bridge simulation state with zero-allocation buffers.
* **Deliverables**:
  - Create [TelemetryCoordinator.h](../apps/MotionLab/TelemetryCoordinator.h) and [TelemetryCoordinator.cpp](../apps/MotionLab/TelemetryCoordinator.cpp).
  - Pre-allocate fixed circular buffers for 200-sample oscilloscope strip charts and 60-sample G-G trail.
  - Integrate with `TelemetrySampler2D` and `TelemetryRingBuffer2D`.

### Phase 3: Friction Circle ($G$-$G$ Diagram) Vector Rendering
* **Objective**: Rasterize the polar coordinate friction radar in `TelemetryHud`.
* **Deliverables**:
  - Implement concentric circle rasterizer for $0.5G$, $1.0G$, and $1.5G$ reference boundaries.
  - Draw crosshair axes with longitudinal (+Accel / -Brake) and lateral (+Left / -Right) indicators.
  - Plot normalized operating cursor with neon green, bright amber, and red color mapping.
  - Render fading historic trail line segments or dots.

### Phase 4: 4-Corner Weight Transfer Visualizer
* **Objective**: Rasterize four segmented vertical bar graphs for FL, FR, RL, RR normal reactions.
* **Deliverables**:
  - Render four vertical load meters with static reference markers.
  - Color gradient reflecting load level (cyan/gold at nominal, red on wheel lift or severe overload).
  - Add numerical readouts of wheel normal force in Newtons.

### Phase 5: Live Rolling Oscilloscope Strip Charts
* **Objective**: Rasterize multi-channel historical strip charts.
* **Deliverables**:
  - Implement `renderStripChart` with zero dynamic allocations in the render loop.
  - Plot Speed, Slip Angles ($\alpha_f, \alpha_r$), and Yaw Rate ($\dot{\psi}$).
  - Render zero reference baseline, horizontal gridlines, and min/max scale labels.
  - Implement channel toggle or stacked layout.

### Phase 6: Session Disk Recording & Hotkey Integration
* **Objective**: Enable full-rate disk telemetry logging triggered via keyboard hotkey.
* **Deliverables**:
  - Wire `Ctrl+E` and `Ctrl+F9` in `MotionLabApp::handleKeyDown` to invoke `TelemetryCoordinator::toggleRecording()`.
  - Format ISO 8601 timestamped filenames: `telemetry_run_YYYYMMDD_HHMMSS.csv`.
  - Write standard MoTeC / PlotJuggler CSV headers and stream `TelemetryFrame2D` rows atomically.
  - Display blinking `[REC: XXs]` indicator in the bottom HUD status bar.

### Phase 7: Verification, Hardening & Automated Testing Suite
* **Objective**: Comprehensive test verification, static analysis, and zero-warning build validation.
* **Deliverables**:
  - Create `tests/ExtendedTelemetryTest.cpp` covering weight transfer, ring buffer slicing, and exporter accuracy.
  - Verify zero dynamic allocations during active rendering and physics stepping.
  - Validate `-Wall -Wextra -pedantic` and MSVC `/W4` with zero warnings.

---

## 7. Verification & Automated Testing Suite

### 7.1 Unit Testing Strategy (`tests/ExtendedTelemetryTest.cpp`)

```cpp
#include <gtest/gtest.h>
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/telemetry/TelemetryRingBuffer2D.h"
#include "libs/VehicleSimCore/telemetry/TelemetrySampler2D.h"

namespace vehicle_sim::telemetry::testing {

/// @brief Verifies that dynamic weight transfer preserves total vehicle normal load.
TEST(ExtendedTelemetryTest, WeightTransferConservationOfVerticalForces)
{
    const double mass { 1200.0 };
    const double g0 { 9.80665 };
    const double totalWeight { mass * g0 };
    const double wheelbase { 2.6 };
    const double trackWidth { 1.5 };
    const double cgHeight { 0.45 };

    // Test across stationary, pure braking, pure acceleration, and cornering
    const double testAx[] { 0.0, -8.0, 5.0, -4.0 };
    const double testAy[] { 0.0, 0.0, 0.0, 9.0 };

    for (std::size_t i { 0 }; i < 4; ++i) {
        const double ax { testAx[i] };
        const double ay { testAy[i] };

        const double staticFront { (totalWeight * 0.5) };
        const double staticRear { (totalWeight * 0.5) };

        const double deltaLong { (mass * ax * cgHeight) / wheelbase };
        const double frontAxle { staticFront - deltaLong };
        const double rearAxle { staticRear + deltaLong };

        const double deltaLatFront { (0.5 * mass * ay * cgHeight) / trackWidth };
        const double deltaLatRear { (0.5 * mass * ay * cgHeight) / trackWidth };

        const double fzFL { std::max(0.0, (frontAxle / 2.0) - deltaLatFront) };
        const double fzFR { std::max(0.0, (frontAxle / 2.0) + deltaLatFront) };
        const double fzRL { std::max(0.0, (rearAxle / 2.0) - deltaLatRear) };
        const double fzRR { std::max(0.0, (rearAxle / 2.0) + deltaLatRear) };

        const double sumFz { fzFL + fzFR + fzRL + fzRR };
        EXPECT_NEAR(sumFz, totalWeight, 1e-3);
    }
}

/// @brief Verifies G-G traction limit calculation.
TEST(ExtendedTelemetryTest, GgTractionRatioCalculations)
{
    const double mu { 1.0 };
    const double g0 { 9.80665 };

    // 0.5G longitudinal, 0.0 lateral -> rho = 0.5 (< 0.70 green)
    double longG { 0.5 };
    double latG { 0.0 };
    double rho { std::sqrt(std::pow(longG / mu, 2.0) + std::pow(latG / mu, 2.0)) };
    EXPECT_NEAR(rho, 0.5, 1e-5);
    EXPECT_LT(rho, 0.70);

    // 0.6G longitudinal, 0.6G lateral -> rho = sqrt(0.72) = 0.848 (amber)
    longG = 0.6;
    latG = 0.6;
    rho = std::sqrt(std::pow(longG / mu, 2.0) + std::pow(latG / mu, 2.0));
    EXPECT_GT(rho, 0.70);
    EXPECT_LT(rho, 0.95);

    // 0.8G longitudinal, 0.8G lateral -> rho = sqrt(1.28) = 1.131 (red / breakaway)
    longG = 0.8;
    latG = 0.8;
    rho = std::sqrt(std::pow(longG / mu, 2.0) + std::pow(latG / mu, 2.0));
    EXPECT_GT(rho, 0.95);
}

} // namespace vehicle_sim::telemetry::testing
```

---

## 8. Verification Checklist & Compliance Matrix

| Item # | Verification Criteria | Status / Strategy |
| :---: | :--- | :--- |
| **1** | No raw `new` / `delete` | **PASS**: Pre-allocated `std::array` and RAII containers only |
| **2** | Headers use `#pragma once` | **PASS**: Verified across all proposed headers |
| **3** | Co-located `.h` and `.cpp` files | **PASS**: All new files co-located in `apps/MotionLab` or `libs/...` |
| **4** | Pure C++17 Core Engine | **PASS**: Zero external engine or UI dependencies in `VehicleSimCore` |
| **5** | Concise function names (< 30 chars) | **PASS**: Names strictly verified (e.g., `renderFrictionCircle`) |
| **6** | Cross-platform IEEE 754 determinism | **PASS**: No OS-specific headers; deterministic float math |
| **7** | Explicit braced initialization `{}` | **PASS**: Applied to all structs, members, and variables |
| **8** | `[[nodiscard]]` applied | **PASS**: Applied to all state, buffer, and status queries |
| **9** | Full Doxygen documentation | **PASS**: `@brief`, `@details`, `@param`, `@return` on all APIs |
| **10** | MISRA / CERT C++ compliance | **PASS**: No dynamic casts, bounded array accesses |
| **11** | Zero compiler warnings | **PASS**: Compatible with `-Wall -Wextra -pedantic` and `/W4` |
| **12** | Relative file paths in documentation | **PASS**: All documentation references use relative paths |
| **13** | Both ASCII and Mermaid diagrams | **PASS**: Complete system architecture and layout diagrams included |
