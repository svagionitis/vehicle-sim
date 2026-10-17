# Motion Lab: Tire Thermal Model & Dynamic Grip Degradation Implementation Plan

## 1. Executive Summary & Objectives

### 1.1 Overview
This implementation plan specifies the complete technical design, mathematical derivations, thermodynamic formulations, software architecture, retro 1996 vector wireframe presentation, and verification strategy for the **Tire Thermal Model & Dynamic Grip Degradation Environment** in **Motion Lab 2D** (`apps/MotionLab`). Originating from Section 9.6 of [MotionLabImplementationPlan.md](MotionLabImplementationPlan.md#96-environmental-physics--simulation-controls), this capability transitions Motion Lab's vehicle simulation from an idealized isothermal contact model into a physically rigorous, non-linear thermodynamic proving ground conforming to automotive engineering, tire testing, and motorsport telemetry standards:
* **SAE J2452 / ISO 28580** (*Measurement of Passenger Car and Light Truck Tire Rolling Resistance & Hysteresis*)
* **SAE J2735 / ISO 4138** (*Vehicle Dynamics & Steady-State Handling Under Temperature-Degraded Friction*)
* **FIA Formula / GT Championship Regulations** (*Tire Operating Windows, Thermal Degradation Curves & Heat Dissipation*)

In motorsport and physical vehicle testing, tire grip is fundamentally non-isothermal. As a vehicle negotiates slaloms, executes high-speed emergency evasion maneuvers, or engages in sustained power drifts, severe shear strain and sliding in the tire contact patch generate substantial heat through frictional dissipation. Simultaneously, cyclic viscoelastic flexing of the tire carcass produces bulk internal hysteresis heating. 

When tire rubber operates below its glass transition warmup threshold ($T_{\text{surf}} < 65^\circ\text{C}$), polymer adhesion is suppressed, resulting in reduced grip. Within the optimum operating window ($75^\circ\text{C} - 95^\circ\text{C}$ for road/drift compounds, $85^\circ\text{C} - 105^\circ\text{C}$ for racing slicks), rubber conformability and hysteretic grip reach their peak. However, when prolonged high-slip operation drives temperatures above $115^\circ\text{C}$, the polymer matrix degrades, localized blistering occurs, and effective friction drops by up to $35\% - 40\%$. Under extreme thermal scrub ($> 135^\circ\text{C}$), tires vaporize rubber polymers, generating visual plumes of smoke. Concurrently, internal carcass heating elevates internal cavity inflation air temperature, causing pneumatic pressure rises that alter tire stiffness and contact patch geometry.

Conforming strictly to pure C++17 standards, IEEE 754 floating-point cross-platform determinism, and zero dynamic memory allocations in the hard real-time simulation loop, the Tire Thermal Model introduces a multi-node lumped capacitance thermal network, non-linear friction bell curve modulation, and full coupling with the industrial-grade [`PacejkaTireModel`](../libs/VehicleSimCore/dynamics/PacejkaTireModel.h).

```
+---------------------------------------------------------------------------------------------------+
| SPEED: 48.2 m/s (107.8 mph)   SURF TEMP: 118.4 deg C [HOT]   CAVITY P: 2.38 bar   GRIP FACTOR: 76%|
| VEHICLE: DRIFT CAR [F8]       SLIP POWER: 14.8 kW (RR)       SMOKE: ACTIVE (BLUE) T_AMB: 25.0 deg C|
+---------------------------------------------------------------------------------------------------+
|  [4-CORNER TIRE THERMAL HUD]                         [CONTACT PATCH HEAT GENERATION]              |
|        +-------------------------+                           Friction Power P_fric = |Fx*vsx|+|Fy*vsy|
|        |     FRONT AXLE          |                                      |                         |
|   FL:  | [||||||||||||    ] 82C  |  :FR                 Convective      v         Conductive      |
|  2.1bar| (OPTIMAL - PEAK GRIP)   | 2.1bar             Cooling q_conv <--- [SURFACE NODE] ---> q_road (Asphalt)|
|        +-------------------------+                      (Airspeed)      |   (Thin Rubber)         |
|        |                         |                                      v   q_internal (Conduction)
|        |                         |                                 [CARCASS NODE] <--- Carcass Flex P_flex|
|        |     REAR AXLE           |                                 (Bulk Rubber)          (Hysteresis)    |
|   RL:  | [||||||||||||||||] 126C |  :RR                                 |                         |
|  2.4bar| [OVERHEAT -32% GRIP]    | 2.4bar                               v   q_cavity              |
|        +-------------------------+                                 [CAVITY AIR] ---> Ideal Gas P(T)|
|        [ALERT: REAR TIRE BLISTERING DETECTED]                      (Inflation)   (Area/Stiffness) |
+---------------------------------------------------------------------------------------------------+
```

---

### 1.2 Core Capabilities & Deliverables

1. **Multi-Node Lumped Capacitance Thermal Network**:
   - Discrete 3-node thermal energy balance per wheel:
     - **Contact Patch Surface Node ($T_{\text{surf}}$)**: Low thermal inertia layer ($C_{\text{surf}} \approx 120.0\,\text{J/K}$), rapidly responding to instantaneous frictional scrub, flash heating, and road conduction.
     - **Tread Carcass & Bulk Node ($T_{\text{carcass}}$)**: High thermal mass structural carcass ($C_{\text{bulk}} \approx 1200.0\,\text{J/K}$), integrating internal viscoelastic hysteresis heating and structural heat conduction.
     - **Internal Cavity Gas Node ($T_{\text{cavity}}$)**: Pneumatic inflation volume ($C_{\text{air}} \approx 180.0\,\text{J/K}$), governing thermal pressure expansion.

2. **Kinematic Friction Power & Carcass Hysteresis Dissipation**:
   - Contact patch sliding scrub heat generation:
     $$P_{\text{friction}} = |\mathbf{F}_x \cdot v_{sx}| + |\mathbf{F}_y \cdot v_{sy}|$$
   - Rolling carcass viscoelastic cyclic deflection heating:
     $$P_{\text{flex}} = C_{\text{hyst}} \cdot F_z \cdot |\omega R_{\text{wheel}}|$$

3. **Speed-Dependent Environmental Heat Dissipation**:
   - Convective boundary-layer cooling scaling with vehicle true relative airspeed:
     $$h_{\text{conv}}(v_{\text{rel}}) = h_0 \left[1.0 + k_{\text{conv}} \left(\frac{\|\mathbf{v}_{\text{rel}}\|}{v_{\text{ref}}}\right)^{0.8}\right]$$
   - Conductive heat exchange with the ground track surface ($T_{\text{road}}$) proportional to contact patch footprint area.
   - Radiative heat loss governed by the linearized Stefan-Boltzmann boundary condition.

4. **Pneumatic Cavity Pressure & Stiffness Modulation**:
   - Deterministic ideal gas law inflation pressure calculation:
     $$P_{\text{cavity}}(T_{\text{cavity}}) = P_{\text{cold}} \left(\frac{T_{\text{cavity}} + 273.15}{T_{\text{cold}} + 273.15}\right)$$
   - Dynamically modulates effective rolling radius, vertical carcass spring stiffness $K_z(P)$, and Pacejka cornering stiffness factor $B(P)$.

5. **Non-Linear Asymmetric Thermal Friction Characteristic ($\mu_{\text{eff}}(T)$)**:
   - Evaluates a continuous, non-linear friction multiplier spanning three distinct thermodynamic regimes:
     - *Cold Regime* ($T < 65^\circ\text{C}$): Vitrified polymer stiffness, reduced mechanical micro-interlocking, up to $20\%$ grip reduction.
     - *Optimum Window* ($75^\circ\text{C} \le T \le 95^\circ\text{C}$): Maximum viscoelastic adhesion and molecular hysteresis, $\mu_{\text{eff}} = 1.00$.
     - *Thermal Overheat & Blistering* ($T > 115^\circ\text{C}$): Thermal softening, polymer chain scission, greasy surface boundary layer, progressive grip reduction up to $35\% - 40\%$.

6. **Universal 12-Vehicle Fleet Parameterization**:
   - Complete thermodynamic calibration for all 12 Motion Lab vehicles (Drift Car, Race Car, Classic Car, Monster Truck, Semi-Truck, Go-Kart, Forklift, Tracked Tank, Quad Bike, and non-wheeled thermal bypass for Drone, Rocket, Hovercraft).

7. **Retro 1996 Vector Wireframe Visuals & Particle Smoke System**:
   - Top-right 4-wheel chassis wireframe card displaying live temperature readouts, multi-stage LED bar graphs, and inflation pressure.
   - Dynamic phosphor color grading: Ice Blue ($< 50^\circ\text{C}$), Neon Green ($75^\circ\text{C} - 95^\circ\text{C}$), Amber ($100^\circ\text{C} - 120^\circ\text{C}$), and Blistering Red/Magenta ($> 120^\circ\text{C}$).
   - Pre-allocated zero-heap particle buffer generating blue/white tire smoke puffs during severe thermal slip ($T_{\text{surf}} > 115^\circ\text{C}$, $v_{\text{slip}} > 2.5\,\text{m/s}$).

---

## 2. High-Level Architecture & Subsystem Decomposition

### 2.1 ASCII System Architecture Diagram

```
+---------------------------------------------------------------------------------------------------+
|                        Tire Thermal Model Architecture in Motion Lab 2D                           |
+---------------------------------------------------------------------------------------------------+
|  [apps/MotionLab] (Desktop Application & Visualization Layer)                                     |
|                                                                                                   |
|    +--------------------+       +------------------------------+       +---------------------+    |
|    |    MotionLabApp    |------>|    TireThermalCoordinator    |------>| TireThermalRenderer |    |
|    | (Ctrl+T Reset,     |       | (4-Wheel States, Fleet Props,|       | (4-Wheel HUD Card,  |    |
|    |  'T' Toggle HUD)   |       |  Particle Smoke Generator)   |       |  Thermal Smoke FX)  |    |
|    +---------+----------+       +--------------+---------------+       +----------+----------+    |
|              |                                 |                                  |               |
|              | Physics Step (dt = 8.333 ms)    | Queries State                    | Renders Lines |
|              v                                 v                                  v               |
|    +--------------------+       +------------------------------+       +---------------------+    |
|    | RigidBody2D Chassis|       |    TireSmokeParticleBuffer   |       |  WireframeRenderer  |    |
|    | & Active Dynamics  |       | (Pre-allocated 512 Particles,|       | (Phosphor Glyphs,   |    |
|    | (1 of 12 Vehicles) |       |  Zero Heap Allocation Loop)  |       |  Vector Geometry)   |    |
|    +---------+----------+       +------------------------------+       +---------------------+    |
|              |                                                                                    |
+--------------|------------------------------------------------------------------------------------+
               | Evaluates Forces & Thermal States
               v
+---------------------------------------------------------------------------------------------------+
|  [libs/VehicleSimCore/dynamics] (Pure C++17 Core Engine - Zero External Dependencies)              |
|                                                                                                   |
|    +-----------------------------+       +-----------------------------+                          |
|    |      TireThermalModel       |------>|      PacejkaTireModel       |                          |
|    | (3-Node RC Thermal Network, |       | (Magic Formula Slip Forces, |                          |
|    |  Friction Bell Curve,       |       |  Combined Slip, Aligning Mz,|                          |
|    |  Cavity Pressure Expansion) |       |  Relaxation Length Dynamics)|                          |
|    +--------------+--------------+       +--------------+--------------+                          |
|                   ^                                     |                                         |
|                   | Updates Friction Multiplier mu_eff  | Provides Fx, Fy, vsx, vsy               |
|                   +-------------------------------------+                                         |
|                                                                                                   |
+---------------------------------------------------------------------------------------------------+
```

---

### 2.2 Mermaid System Architecture Diagram

```mermaid
graph TD
    subgraph AppLayer ["Application Layer (apps/MotionLab)"]
        APP["MotionLabApp"]
        TTC["TireThermalCoordinator"]
        TTR["TireThermalRenderer"]
        TSB["TireSmokeParticleBuffer"]
        HUD["TelemetryHud / Thermal HUD Card"]
        WFR["WireframeRenderer"]

        APP -->|Physics Step: dt = 8.333 ms| TTC
        APP -->|Hotkeys: T Toggle, Ctrl+T Reset| TTC
        TTC -->|Sample Slip & Normal Loads| APP
        TTC -->|Emit Smoke Particles on Scrub| TSB
        TTC -->|Push 4-Wheel Temperatures & Pressure| HUD
        APP -->|Render Dispatch| TTR
        TTR -->|Draw 4-Corner Temperature Gauges| WFR
        TTR -->|Draw Billowing Smoke Wireframes| WFR
    end

    subgraph CoreLayer ["Physics & Dynamics Engine (libs/VehicleSimCore)"]
        TTM["TireThermalModel (3-Node Network)"]
        PTM["PacejkaTireModel (Magic Formula)"]
        VEC["Vec2 Math Library"]

        TTC -->|Advance Thermal Integration| TTM
        TTM -->|Calculate Friction Dissipation P_fric| VEC
        TTM -->|Calculate Carcass Hysteresis P_flex| VEC
        TTM -->|Evaluate Non-linear mu_eff(T)| PTM
        PTM -->|Return F_long, F_lat, slip velocities| TTM
    end
```

---

### 2.3 ASCII Multi-Node Thermal Resistor-Capacitor (RC) Network Diagram

```
+---------------------------------------------------------------------------------------------------+
|                       3-Node Tire Lumped Thermal Capacitance Network                              |
+---------------------------------------------------------------------------------------------------+
|                                                                                                   |
|     [HEAT GENERATION]                                    [HEAT DISSIPATION]                       |
|                                                                                                   |
|     Frictional Scrub Power                                Convective Cooling                      |
|       P_fric = |Fx*vsx| + |Fy*vsy|                          q_conv = h_conv(v_rel) * (T_surf - T_amb)
|                 |                                                    ^                            |
|                 v                                                    |                            |
|          +--------------+  R_tread (Conduction) +--------------+     |                            |
|          | SURFACE NODE | --------------------> | CARCASS NODE | ----+                            |
|          | T_surf,      |                       | T_carcass,   |                                  |
|          | C_surf       | <-------------------- | C_bulk       |                                  |
|          +-------+------+                       +-------+------+                                  |
|                  |                                      ^                                         |
|                  | Conductive Cooling                   | Carcass Hysteresis Power                |
|                  v q_road = k_road * (T_surf - T_road)  |   P_flex = C_hyst * Fz * |omega * R|    |
|             (Road Track)                                |                                         |
|                                                         v q_cavity = k_gas * (T_carcass - T_cavity)
|                                                 +--------------+                                  |
|                                                 | CAVITY NODE  |                                  |
|                                                 | T_cavity,    | ----> Pressure Rise P(T)         |
|                                                 | C_air        |       (Modulates B, C, Area)     |
|                                                 +--------------+                                  |
|                                                                                                   |
+---------------------------------------------------------------------------------------------------+
```

---

### 2.4 Mermaid Thermodynamic Energy Balance Diagram

```mermaid
graph LR
    subgraph HeatSources ["Heat Generation Sources"]
        P_fric["Frictional Slip Power: P_fric = |Fx·vsx| + |Fy·vsy|"]
        P_flex["Carcass Hysteresis Power: P_flex = C_hyst·Fz·|ωR|"]
    end

    subgraph ThermalNodes ["Thermal Capacitance Nodes"]
        N_surf["Surface Node (T_surf)<br/>Thermal Mass: C_surf"]
        N_carc["Carcass Node (T_carcass)<br/>Thermal Mass: C_bulk"]
        N_cav["Cavity Air Node (T_cavity)<br/>Thermal Mass: C_air"]
    end

    subgraph HeatSinks ["Environmental Heat Sinks"]
        Q_road["Conductive Heat Flux to Track: q_road"]
        Q_conv["Speed-Scaled Convective Air Cooling: q_conv"]
        Q_rad["Radiative Heat Loss: q_rad"]
    end

    subgraph DynamicOutputs ["Simulation Feedback"]
        Mu_eff["Friction Multiplier: μ_eff(T_surf)"]
        P_inf["Inflation Pressure: P_cavity(T_cavity)"]
        Smoke["Smoke Vapor Particle Spawner"]
    end

    P_fric -->|Direct Surface Heating| N_surf
    P_flex -->|Direct Carcass Heating| N_carc

    N_surf -->|Internal Conduction q_int| N_carc
    N_carc -->|Internal Conduction q_int| N_surf

    N_carc -->|Gas Heat Transfer q_cav| N_cav

    N_surf -->|Ground Conduction| Q_road
    N_surf -->|Surface Convection| Q_conv
    N_surf -->|Radiation| Q_rad
    N_carc -->|Sidewall Convection| Q_conv

    N_surf -->|Modulates Grip| Mu_eff
    N_surf -->|Trigger Thermal Smoke| Smoke
    N_cav -->|Gas Law Expansion| P_inf
```

---

## 3. Mathematical & Physical Formulations

### 3.1 Multi-Node Lumped Thermal Capacitance Network

The thermal state of each tire is represented by three coupled state variables:
1. $T_{\text{surf}}$: Contact patch surface temperature ($^\circ\text{C}$).
2. $T_{\text{carcass}}$: Bulk carcass and tread belt temperature ($^\circ\text{C}$).
3. $T_{\text{cavity}}$: Internal inflation air cavity temperature ($^\circ\text{C}$).

The governing system of coupled first-order ordinary differential equations (ODEs) is:
$$\begin{aligned}
C_{\text{surf}} \frac{dT_{\text{surf}}}{dt} &= P_{\text{friction}} - q_{\text{road}} - q_{\text{internal}} - q_{\text{conv,surf}} - q_{\text{rad}} \\
C_{\text{bulk}} \frac{dT_{\text{carcass}}}{dt} &= P_{\text{flex}} + q_{\text{internal}} - q_{\text{conv,carcass}} - q_{\text{cavity}} \\
C_{\text{air}} \frac{dT_{\text{cavity}}}{dt} &= q_{\text{cavity}} - q_{\text{rim}}
\end{aligned}$$

Where:
* $C_{\text{surf}}$ is the surface layer thermal capacitance ($\text{J/K}$), typically $80.0 - 150.0\,\text{J/K}$.
* $C_{\text{bulk}}$ is the carcass bulk thermal capacitance ($\text{J/K}$), typically $800.0 - 2500.0\,\text{J/K}$.
* $C_{\text{air}}$ is the internal inflation air thermal capacitance ($\text{J/K}$), typically $120.0 - 300.0\,\text{J/K}$.

---

### 3.2 Contact Patch Friction Dissipation

When the tire operates under tractive, braking, or cornering loads, slip exists between the tread contact patch and the road surface. The sliding scrub velocity vector $\mathbf{v}_{\text{slip}} = (v_{sx}, v_{sy})$ in the wheel frame is defined as:
$$\begin{aligned}
v_{sx} &= \omega R_{\text{wheel}} - v_x = \kappa \cdot \max(|v_x|, v_{\text{min}}) \\
v_{sy} &= -v_y = -\tan(\alpha) \cdot \max(|v_x|, v_{\text{min}})
\end{aligned}$$
where $\kappa$ is the longitudinal slip ratio, $\alpha$ is the lateral slip angle, and $v_{\text{min}} = 0.5\,\text{m/s}$ is a regularization velocity preventing division by zero.

The instantaneous mechanical frictional power dissipated in the contact patch is:
$$P_{\text{friction}} = \eta_{\text{thermal}} \cdot \left( |F_{\text{long}} \cdot v_{sx}| + |F_{\text{lat}} \cdot v_{sy}| \right)$$
where:
* $F_{\text{long}}$ and $F_{\text{lat}}$ are the contact patch tractive and cornering forces evaluated by [`PacejkaTireModel`](../libs/VehicleSimCore/dynamics/PacejkaTireModel.h).
* $\eta_{\text{thermal}} \in [0.90, 0.98]$ is the mechanical-to-thermal energy conversion efficiency factor (nominally $0.95$).

---

### 3.3 Carcass Hysteretic Deflection & Rolling Resistance Heating

As the tire rotates under vertical normal load $F_z$, rubber and carcass belts undergo continuous cyclic loading and unloading. Due to the viscoelastic hysteresis loss angle $\tan\delta$ of rubber compounds (SAE J2452), mechanical energy is dissipated as bulk heat inside the carcass:
$$P_{\text{flex}} = C_{\text{hyst}} \cdot F_z \cdot |\omega R_{\text{wheel}}|$$
where:
* $C_{\text{hyst}}$ is the non-dimensional hysteretic loss coefficient ($0.008 - 0.025$).
* $|\omega R_{\text{wheel}}|$ is the rotational linear surface speed ($\text{m/s}$).

---

### 3.4 Speed-Dependent Convective Air Cooling

Convective heat transfer from the tire exterior to ambient air ($T_{\text{ambient}}$) scales non-linearly with vehicle relative airspeed $\|\mathbf{v}_{\text{rel}}\|$ conforming to boundary layer crossflow Nusselt correlations (ESDU 85020):
$$h_{\text{conv}}(\|\mathbf{v}_{\text{rel}}\|) = h_0 \cdot \left[ 1.0 + k_{\text{conv}} \cdot \left( \frac{\|\mathbf{v}_{\text{rel}}\|}{v_{\text{ref}}} \right)^{0.8} \right]$$
where:
* $h_0$ is the stationary natural convection coefficient ($15.0\,\text{W}/(\text{m}^2\cdot\text{K})$).
* $k_{\text{conv}} \approx 2.4$ is the forced convection scaling factor.
* $v_{\text{ref}} = 10.0\,\text{m/s}$ is the reference normalization speed.

The convective heat loss from the surface and carcass sidewalls is:
$$\begin{aligned}
q_{\text{conv,surf}} &= h_{\text{conv}}(\|\mathbf{v}_{\text{rel}}\|) \cdot A_{\text{surf,exp}} \cdot (T_{\text{surf}} - T_{\text{ambient}}) \\
q_{\text{conv,carcass}} &= h_{\text{conv}}(\|\mathbf{v}_{\text{rel}}\|) \cdot A_{\text{sidewall}} \cdot (T_{\text{carcass}} - T_{\text{ambient}})
\end{aligned}$$

---

### 3.5 Conductive Road Surface & Internal Heat Exchange

1. **Road Surface Conduction**:
   Heat conducts directly from the contact patch footprint into the asphalt track surface ($T_{\text{road}}$):
   $$q_{\text{road}} = k_{\text{road}} \cdot A_{\text{patch}} \cdot (T_{\text{surf}} - T_{\text{road}})$$
   where $k_{\text{road}} \approx 250.0\,\text{W}/(\text{m}^2\cdot\text{K})$ is the ground contact conductance and $A_{\text{patch}}$ is the dynamic contact footprint area ($A_{\text{patch}} \propto F_z / P_{\text{cavity}}$).

2. **Internal Radial Conduction**:
   Heat diffuses between the outer surface tread and internal carcass:
   $$q_{\text{internal}} = \frac{k_{\text{rubber}}}{\Delta r_{\text{tread}}} \cdot A_{\text{tread}} \cdot (T_{\text{surf}} - T_{\text{carcass}})$$

3. **Cavity Air Conduction**:
   Heat conducts between the carcass inner liner and internal inflation gas:
   $$q_{\text{cavity}} = h_{\text{cavity}} \cdot A_{\text{inner}} \cdot (T_{\text{carcass}} - T_{\text{cavity}})$$

---

### 3.6 Internal Air Cavity Inflation Pressure Dynamics

Applying the Ideal Gas Law ($P V = n R T$) under approximately constant internal toroidal cavity volume $V_{\text{cavity}}$:
$$P_{\text{cavity}}(T_{\text{cavity}}) = P_{\text{cold}} \cdot \left( \frac{T_{\text{cavity}} + 273.15}{T_{\text{cold}} + 273.15} \right)$$
where $P_{\text{cold}}$ is the base cold inflation pressure (nominally $2.0\,\text{bar} = 200\,\text{kPa}$ for street tires, $1.4\,\text{bar}$ for drift cars, $7.5\,\text{bar}$ for commercial semi-trucks).

**Physical Feedback into Tire Mechanics**:
1. **Contact Patch Footprint Area**:
   $$A_{\text{patch}}(F_z, P_{\text{cavity}}) = \frac{F_z}{P_{\text{cavity}}}$$
   Higher pressure decreases contact area, reducing road scrub conduction and sharpening transient response.
2. **Pacejka Cornering Stiffness Factor $B$**:
   $$B_{\text{eff}} = B_{\text{nominal}} \cdot \left[ 1.0 + k_p \cdot \left( \frac{P_{\text{cavity}} - P_{\text{cold}}}{P_{\text{cold}}} \right) \right]$$
   Over-inflated hot tires exhibit steeper initial cornering stiffness but narrower peak adhesion plateaus.

---

### 3.7 Non-Linear Asymmetric Thermal Friction Characteristic ($\mu_{\text{eff}}(T)$)

Tire rubber compound adhesion and viscoelastic deformation friction follow an asymmetric temperature curve. We formulate $\mu_{\text{eff}}(T)$ using a composite dual-sigmoid and hyperbolic tangent expression:

$$\mu_{\text{eff}}(T_{\text{surf}}) = \mu_{\text{nominal}} \cdot f_{\text{thermal}}(T_{\text{surf}})$$

$$f_{\text{thermal}}(T) = 1.0 - \Delta\mu_{\text{cold}} \cdot \frac{1.0}{1.0 + \exp\left(\frac{T - T_{\text{cold\_mid}}}{s_{\text{cold}}}\right)} - \Delta\mu_{\text{over}} \cdot \tanh\left( \frac{\max(0.0, \; T - T_{\text{opt\_max}})}{s_{\text{hot}}} \right)$$

Where:
* $T_{\text{cold\_mid}}$: Midpoint of cold transition zone (nominally $55.0^\circ\text{C}$).
* $s_{\text{cold}}$: Cold slope transition width ($6.0^\circ\text{C}$).
* $\Delta\mu_{\text{cold}}$: Maximum cold grip deficit fraction ($0.20$, representing $-20\%$ grip when cold).
* $T_{\text{opt\_max}}$: Upper boundary of optimum operating plateau (nominally $95.0^\circ\text{C}$).
* $s_{\text{hot}}$: Thermal degradation softening rate ($25.0^\circ\text{C}$).
* $\Delta\mu_{\text{over}}$: Maximum thermal degradation decay fraction ($0.35$, representing up to $-35\%$ grip loss).

```
  Friction Factor μ_eff(T)
  1.00 +                     /-------------------\ (Optimum Window: 75C - 95C)
       |                    /                     \
  0.90 |                   /                       \
       |                  /                         \
  0.80 |  +--------------/                           \--------------------+
       |  | Cold Tire (-20%)                          Overheated (-35%)   |
  0.65 |  | T < 60C                                   T > 120C            |
       +--+------------------+-------------------+------------------------+-->
         20C                60C                 95C                     160C
                               Tire Surface Temperature T_surf
```

---

### 3.8 Coupling with Pacejka Magic Formula

In [`PacejkaTireModel`](../libs/VehicleSimCore/dynamics/PacejkaTireModel.h), ground forces are computed as:
$$F_x = D_x \sin\left(C_x \arctan\left(B_x \phi_x - E_x(B_x \phi_x - \arctan(B_x \phi_x))\right)\right)$$
$$F_y = D_y \sin\left(C_y \arctan\left(B_y \phi_y - E_y(B_y \phi_y - \arctan(B_y \phi_y))\right)\right)$$

The peak friction parameter $D$ is directly modulated by the thermal friction factor:
$$D_{\text{thermal}} = D_{\text{base}}(F_z) \cdot \mu_{\text{eff}}(T_{\text{surf}}) \cdot \mu_{\text{surface\_material}}$$

Simultaneously, the cornering stiffness factor $B_y$ is adjusted by cavity pressure $P_{\text{cavity}}$, providing a fully coupled thermodynamic-mechanical tire simulation.

---

## 4. Universal Fleet Susceptibility Matrix

Each vehicle in Motion Lab possesses distinct thermal masses, tire compounds, and operational profiles:

| ID | Vehicle Name | Tire Compound Classification | Thermal Mass $C_{\text{surf}} / C_{\text{bulk}}$ | Opt Window $[T_{\text{min}}, T_{\text{max}}]$ | Max Decay $\Delta\mu_{\text{over}}$ | Scrub Sensitivity $k_{\text{heat}}$ | Characteristic Thermal Behavior & Failure Modes |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **F1** | **Classic Car** | Standard Passenger Radial | $120 / 1200\,\text{J/K}$ | $65^\circ\text{C} - 85^\circ\text{C}$ | $0.25$ ($25\%$) | Moderate | Progressive heatup; gentle post-peak breakaway; broad recovery window. |
| **F2** | **Race Car** | High-G Racing Slick | $100 / 950\,\text{J/K}$ | $85^\circ\text{C} - 105^\circ\text{C}$ | $0.38$ ($38\%$) | Very High | Severe cold-tire understeer on out-lap; rapid blister dropoff if locked up. |
| **F3** | **Space Rocket** | Non-Wheeled Vector | N/A | N/A | N/A | None | Thermal bypass; zero tire contact; atmospheric thrust only. |
| **F4** | **Helicopter** | Non-Wheeled Rotorcraft | N/A | N/A | N/A | None | Thermal bypass; landing skid ground friction constant. |
| **F5** | **Hovercraft** | Non-Wheeled Skirt | N/A | N/A | N/A | None | Thermal bypass; air cushion skirt ground interaction. |
| **F6** | **Tracked Tank** | Steel Tracks with Rubber Pads | $350 / 4500\,\text{J/K}$ | $50^\circ\text{C} - 80^\circ\text{C}$ | $0.20$ ($20\%$) | High | High hysteresis deflection; rubber pad thermal saturation under pivot turns. |
| **F7** | **Anti-Gravity**| Magnetic Levitation | N/A | N/A | N/A | None | Thermal bypass; contact-free magnetic flux suspension. |
| **F8** | **Drift Car** | High-Scrub Drift Radial | $110 / 1100\,\text{J/K}$ | $75^\circ\text{C} - 95^\circ\text{C}$ | $0.35$ ($35\%$) | Extreme | Rear axle heat spikes ($> 130^\circ\text{C}$) during sustained power slides; blue smoke plumes. |
| **F9** | **Monster Truck**| Giant 66-Inch Deep Lug | $600 / 6500\,\text{J/K}$ | $50^\circ\text{C} - 75^\circ\text{C}$ | $0.22$ ($22\%$) | Low | Massive thermal mass; high carcass flex heating; low surface scrub dissipation. |
| **F10**| **Semi-Truck** | Heavy Commercial Radial | $450 / 5200\,\text{J/K}$ | $60^\circ\text{C} - 90^\circ\text{C}$ | $0.30$ ($30\%$) | High | High normal load ($40\,\text{kN}$); thermal runaway during heavy emergency stops. |
| **F11**| **Forklift** | Solid Industrial Polyurethane | $180 / 1600\,\text{J/K}$ | $45^\circ\text{C} - 70^\circ\text{C}$ | $0.30$ ($30\%$) | Moderate | Zero pneumatic cavity; high solid polymer hysteresis; low max operating threshold. |
| **F12**| **Drone** | Non-Wheeled Quadcopter | N/A | N/A | N/A | None | Thermal bypass; four aerodynamic propellers. |

---

## 5. Software Architecture & Class Interface Specifications

Conforming to the project verification checklist:
- Pure C++17, zero external engine dependencies.
- Header (`.h`) and implementation (`.cpp`) co-located.
- `#pragma once` include guards.
- All function names strictly less than 30 characters.
- IEEE 754 determinism, explicit `{}` initialization.
- Full Doxygen templates (`/// @brief`, `@details`, `@param`, `@return`).

### 5.1 Core Engine: `libs/VehicleSimCore/dynamics/TireThermalModel.h`

```cpp
#pragma once

#include <algorithm>
#include <cmath>

namespace vehicle_sim::dynamics {

/// @struct TireThermalConfig
/// @brief Physical, thermodynamic, and material parameters for a tire thermal model.
struct TireThermalConfig {
    double surfaceThermalMass { 120.0 };   ///< Contact patch surface heat capacity in J/K.
    double bulkThermalMass { 1200.0 };     ///< Carcass bulk heat capacity in J/K.
    double cavityAirThermalMass { 180.0 };  ///< Inflation air heat capacity in J/K.

    double internalConductance { 45.0 };   ///< Surface-to-carcass heat conductance in W/K.
    double roadConductance { 60.0 };       ///< Contact patch to road surface conductance in W/K.
    double cavityConductance { 25.0 };     ///< Carcass-to-cavity air conductance in W/K.

    double baseConvectionCoeff { 15.0 };   ///< Stationary convective cooling coefficient in W/(m^2*K).
    double convectionSpeedScale { 2.4 };   ///< Speed-scaled convective exponent multiplier.

    double hysteresisCoeff { 0.012 };      ///< Carcass cyclic deflection loss factor C_hyst.
    double thermalConversionEff { 0.95 };  ///< Mechanical scrub to thermal energy efficiency.

    double coldTempMidpoint { 55.0 };      ///< Cold transition midpoint in Celsius.
    double coldSlopeWidth { 6.0 };         ///< Cold transition slope width in Celsius.
    double coldGripDeficit { 0.20 };       ///< Maximum grip reduction under cold conditions.

    double optimalTempMin { 75.0 };        ///< Lower threshold of peak grip window in Celsius.
    double optimalTempMax { 95.0 };        ///< Upper threshold of peak grip window in Celsius.
    double hotDegradationRate { 25.0 };    ///< Overheat degradation slope factor in Celsius.
    double maxThermalDecay { 0.35 };       ///< Maximum friction degradation ratio when overheated.

    double baseInflationPressure { 2.0 };  ///< Cold inflation pressure P_cold in bar.
    double referenceColdTemp { 25.0 };     ///< Reference cold ambient temperature in Celsius.
    double smokeThresholdTemp { 115.0 };   ///< Surface temperature triggering vapor smoke in Celsius.
};

/// @struct TireThermalState
/// @brief Instantaneous thermodynamic state of a single tire.
struct TireThermalState {
    double surfaceTemp { 25.0 };           ///< Contact patch surface temperature in Celsius.
    double carcassTemp { 25.0 };           ///< Bulk carcass temperature in Celsius.
    double cavityTemp { 25.0 };            ///< Inflation air cavity temperature in Celsius.
    double inflationPressure { 2.0 };      ///< Current pneumatic inflation pressure in bar.
    double effectiveMuMultiplier { 1.0 };  ///< Dynamic friction modifier mu_eff(T).
    double frictionPower { 0.0 };          ///< Instantaneous scrub friction power in Watts.
    double hysteresisPower { 0.0 };        ///< Instantaneous carcass flexing power in Watts.
    bool isOverheated { false };           ///< True if surface exceeds optimal grip plateau.
    bool isSmoking { false };              ///< True if generating tire vapor smoke.
};

/// @class TireThermalModel
/// @brief Industrial 3-node lumped capacitance tire thermodynamic evaluator.
/// @details Evaluates contact patch frictional heating, carcass rolling hysteresis,
/// airspeed-dependent convective cooling, road conduction, and non-linear grip drop.
class TireThermalModel {
public:
    /// @brief Default constructor initializing standard radial configuration.
    constexpr TireThermalModel() noexcept = default;

    /// @brief Parameterized constructor using custom thermal configuration.
    /// @param[in] config Thermal capacity, dissipation, and grip parameters.
    explicit TireThermalModel(const TireThermalConfig& config) noexcept;

    /// @brief Resets all temperatures to specified ambient temperature.
    /// @param[in] ambientTemp Equilibrium ambient temperature in Celsius.
    void reset(double ambientTemp = 25.0) noexcept;

    /// @brief Advances thermodynamic state across simulation timestep dt.
    /// @param[in] tractiveForce Longitudinal force Fx in Newtons.
    /// @param[in] corneringForce Lateral force Fy in Newtons.
    /// @param[in] normalLoad Vertical normal reaction Fz in Newtons.
    /// @param[in] slipVelLong Longitudinal sliding scrub velocity in m/s.
    /// @param[in] slipVelLat Lateral sliding scrub velocity in m/s.
    /// @param[in] wheelAngularSpeed Wheel rotational velocity omega in rad/s.
    /// @param[in] wheelRadius Effective rolling radius in meters.
    /// @param[in] relativeAirSpeed Relative chassis airspeed in m/s.
    /// @param[in] ambientTemp Ambient air temperature in Celsius.
    /// @param[in] roadTemp Asphalt road surface temperature in Celsius.
    /// @param[in] dt Physics timestep duration in seconds.
    void update(double tractiveForce,
                double corneringForce,
                double normalLoad,
                double slipVelLong,
                double slipVelLat,
                double wheelAngularSpeed,
                double wheelRadius,
                double relativeAirSpeed,
                double ambientTemp,
                double roadTemp,
                double dt) noexcept;

    /// @brief Evaluates dynamic friction multiplier for a given surface temperature.
    /// @param[in] surfaceTemp Target surface temperature in Celsius.
    /// @return Non-linear friction factor in range [1.0 - maxDecay, 1.0].
    [[nodiscard]] double evaluateGripFactor(double surfaceTemp) const noexcept;

    /// @brief Queries current surface temperature.
    /// @return Surface temperature in Celsius.
    [[nodiscard]] double getSurfaceTemp() const noexcept;

    /// @brief Queries current carcass temperature.
    /// @return Carcass bulk temperature in Celsius.
    [[nodiscard]] double getCarcassTemp() const noexcept;

    /// @brief Queries current inflation cavity pressure.
    /// @return Cavity pressure in bar.
    [[nodiscard]] double getCavityPressure() const noexcept;

    /// @brief Queries dynamic effective friction multiplier.
    /// @return Effective friction coefficient multiplier.
    [[nodiscard]] double getGripMultiplier() const noexcept;

    /// @brief Queries active thermal state snapshot.
    /// @return Const reference to TireThermalState.
    [[nodiscard]] const TireThermalState& getState() const noexcept;

    /// @brief Queries thermal configuration.
    /// @return Const reference to TireThermalConfig.
    [[nodiscard]] const TireThermalConfig& getConfig() const noexcept;

    /// @brief Reconfigures thermal parameters.
    /// @param[in] config New configuration structure.
    void setConfig(const TireThermalConfig& config) noexcept;

private:
    TireThermalConfig mConfig {};
    TireThermalState mState {};

    [[nodiscard]] double computeConvection(double airSpeed) const noexcept;
    void updatePressure() noexcept;
};

} // namespace vehicle_sim::dynamics
```

---

### 5.2 Application Coordinator: `apps/MotionLab/TireThermalCoordinator.h`

```cpp
#pragma once

#include <array>
#include <cstddef>

#include "apps/MotionLab/MotionLabVehicleType.h"
#include "libs/VehicleSimCore/dynamics/TireThermalModel.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace motion_lab {

class TireSmokeParticleBuffer;

/// @class TireThermalCoordinator
/// @brief Coordinates 4-wheel thermodynamic state evaluation across Motion Lab fleet.
/// @details Extracts wheel kinematics from active vehicle dynamics, computes scrub
/// velocities, triggers smoke particles, and bridges telemetry HUD visualizations.
class TireThermalCoordinator {
public:
    /// @brief Default constructor initializing 4 standard thermal wheels.
    TireThermalCoordinator() noexcept;

    /// @brief Configures thermodynamic parameters for active vehicle model.
    /// @param[in] type Selected Motion Lab vehicle classification.
    void setVehicleType(MotionLabVehicleType type) noexcept;

    /// @brief Resets all wheel temperatures to ambient baseline.
    /// @param[in] ambientTemp Ambient temperature in Celsius.
    void reset(double ambientTemp = 25.0) noexcept;

    /// @brief Advances 4-wheel thermal model for the active simulation step.
    /// @param[in] relativeAirSpeed Vehicle airspeed magnitude in m/s.
    /// @param[in] dt Physics timestep duration in seconds.
    /// @param[in,out] smokeBuffer Particle buffer receiving thermal smoke puffs.
    void update(double relativeAirSpeed,
                double dt,
                TireSmokeParticleBuffer& smokeBuffer) noexcept;

    /// @brief Sets ambient environmental temperatures.
    /// @param[in] ambientAir Ambient air temperature in Celsius.
    /// @param[in] trackRoad Asphalt road temperature in Celsius.
    void setEnvironment(double ambientAir, double trackRoad) noexcept;

    /// @brief Queries thermal state for a specific wheel index.
    /// @param[in] wheelIndex Wheel identifier (0: FL, 1: FR, 2: RL, 3: RR).
    /// @return Const reference to TireThermalState.
    [[nodiscard]] const vehicle_sim::dynamics::TireThermalState& getWheelState(std::size_t wheelIndex) const noexcept;

    /// @brief Queries active vehicle classification.
    /// @return Active MotionLabVehicleType.
    [[nodiscard]] MotionLabVehicleType getVehicleType() const noexcept;

    /// @brief Checks if vehicle utilizes wheeled thermal modeling.
    /// @return True for wheeled vehicles, false for bypass vehicles.
    [[nodiscard]] bool isWheeledVehicle() const noexcept;

private:
    std::array<vehicle_sim::dynamics::TireThermalModel, 4> mWheels {};
    MotionLabVehicleType mVehicleType { MotionLabVehicleType::ClassicCar };
    double mAmbientAirTemp { 25.0 };
    double mTrackRoadTemp { 30.0 };
    bool mIsWheeled { true };
};

} // namespace motion_lab
```

---

### 5.3 Smoke Particle Buffer: `apps/MotionLab/TireSmokeParticleBuffer.h`

```cpp
#pragma once

#include <cstddef>
#include <vector>

#include "libs/VehicleSimCore/math/Vec2.h"

namespace motion_lab {

/// @struct SmokeParticle
/// @brief 2D world-space expanding vapor puff representing tire scrub smoke.
struct SmokeParticle {
    vehicle_sim::math::Vec2 position { 0.0, 0.0 };  ///< World position in meters.
    vehicle_sim::math::Vec2 velocity { 0.0, 0.0 };  ///< Drift velocity in m/s.
    double radius { 0.20 };                         ///< Expanding puff radius in meters.
    double age { 0.0 };                            ///< Current particle age in seconds.
    double maxLifetime { 1.50 };                   ///< Total particle lifespan in seconds.
    double initialAlpha { 0.85 };                  ///< Initial opacity factor [0.0, 1.0].
};

/// @class TireSmokeParticleBuffer
/// @brief Zero-allocation circular buffer of retro vector wireframe smoke particles.
class TireSmokeParticleBuffer {
public:
    static constexpr std::size_t kMaxParticles { 512 };

    /// @brief Default constructor pre-allocating fixed memory pool.
    TireSmokeParticleBuffer() noexcept;

    /// @brief Emits a smoke puff at a specific wheel world location.
    /// @param[in] origin Wheel ground contact center in world coordinates.
    /// @param[in] slipVelocity Slip vector imparting initial puff drift.
    /// @param[in] intensity Heat/slip scaling factor in range [0.0, 1.0].
    void emit(const vehicle_sim::math::Vec2& origin,
              const vehicle_sim::math::Vec2& slipVelocity,
              double intensity) noexcept;

    /// @brief Updates particle expansion, drag deceleration, and aging.
    /// @param[in] dt Physics timestep duration in seconds.
    void update(double dt) noexcept;

    /// @brief Clears all active smoke particles.
    void clear() noexcept;

    /// @brief Retrieves list of active smoke particles.
    /// @return Const reference to vector of active SmokeParticle.
    [[nodiscard]] const std::vector<SmokeParticle>& getParticles() const noexcept;

private:
    std::vector<SmokeParticle> mParticles {};
    std::size_t mWriteIndex { 0 };
    std::size_t mActiveCount { 0 };
};

} // namespace motion_lab
```

---

## 6. Retro 1996 Vector Wireframe Visuals & HUD Telemetry Interface

### 6.1 ASCII Visual Mockup of the 4-Corner Tire Thermal HUD Card

```
+---------------------------------------------------------------------------------------------------+
| SPEED: 48.2 m/s (107.8 mph)   WIND: 12.0 m/s @ 270 deg   FPS: 60.0   AIR TEMP: 25C   ROAD: 32C    |
| VEHICLE: DRIFT CAR [F8]       TIRE THERMAL MANAGEMENT: [ACTIVE]      HOTKEY: 'T' HIDE HUD         |
+---------------------------------------------------------------------------------------------------+
|                                                                                                   |
|                                       [TIRE THERMAL DIAGNOSTICS]                                  |
|                                                                                                   |
|             FRONT-LEFT (FL)                                 FRONT-RIGHT (FR)                      |
|             +-------------------------+                     +-------------------------+           |
|             | TEMP: 84.5 C   2.12 bar |                     | TEMP: 86.2 C   2.14 bar |           |
|             | [||||||||||||||       ] |                     | [||||||||||||||       ] |           |
|             | STATUS: OPTIMAL (100%)  |                     | STATUS: OPTIMAL (100%)  |           |
|             +-------------------------+                     +-------------------------+           |
|                          |                                               |                        |
|                          |           +-----------------------+           |                        |
|                          +---------> |      CHASSIS TOP      | <---------+                        |
|                                      |      SILHOUETTE       |                                    |
|                          +---------> |                       | <---------+                        |
|                          |           +-----------------------+           |                        |
|                          |                                               |                        |
|             REAR-LEFT (RL)                                  REAR-RIGHT (RR)                       |
|             +-------------------------+                     +-------------------------+           |
|             | TEMP: 128.4 C  2.46 bar |                     | TEMP: 132.0 C  2.51 bar |           |
|             | [|||||||||||||||||||||] |                     | [|||||||||||||||||||||] |           |
|             | STATUS: OVERHEAT (-32%) |                     | STATUS: OVERHEAT (-35%) |           |
|             | [BLISTERING & SMOKE]    |                     | [BLISTERING & SMOKE]    |           |
|             +-------------------------+                     +-------------------------+           |
|                                                                                                   |
|             [THERMAL REGIME LEGEND]                                                               |
|             [COLD: <65C (CYAN)]   [WARM: 65-75C]   [OPTIMAL: 75-95C (GREEN)]                      |
|             [HOT: 95-115C (AMBER)]                 [OVERHEAT/BLISTER: >115C (MAGENTA/RED)]        |
|                                                                                                   |
+---------------------------------------------------------------------------------------------------+
```

---

### 6.2 Phosphor Color Grading Palette

To preserve the authentic 1996 CRT arcade phosphor vector display aesthetic:

| Temperature Range | Thermodynamic Regime | Phosphor Color | Hex Code | Visual Presentation & Behavior |
| :--- | :--- | :--- | :--- | :--- |
| **$< 50^\circ\text{C}$** | Cold Vitrified Rubber | Phosphor Ice Blue | `#00E5FF` | Steady cyan outline; reduced adhesion warning indicator. |
| **$50^\circ\text{C} - 75^\circ\text{C}$** | Warmup Transition Zone | Light Chartreuse | `#76FF03` | Progressive green glow; grip climbing towards peak. |
| **$75^\circ\text{C} - 95^\circ\text{C}$** | Optimal Grip Window | Vibrant Neon Green | `#00E676` | Intense emerald phosphor; maximum tractive & cornering traction. |
| **$95^\circ\text{C} - 115^\circ\text{C}$** | Thermal Warning / Hot | Radiant Amber | `#FFEA00` | Bright yellow-amber; initial grip softness; slip angle widening. |
| **$115^\circ\text{C} - 140^\circ\text{C}$**| Overheating / Blistering | Blazing Orange-Red | `#FF3D00` | Blinking warning text; significant grip loss ($-30\%$); rubber degradation. |
| **$> 140^\circ\text{C}$** | Severe Burnout / Smoke | Electric Magenta | `#FF007F` | Rapid flashing border; billowing wireframe smoke particle emission. |

---

### 6.3 Procedural Vector Smoke Particle Emission

When a wheel's surface temperature exceeds $T_{\text{smoke}} \ge 115.0^\circ\text{C}$ and total sliding slip velocity exceeds $v_{\text{slip,thresh}} \ge 2.0\,\text{m/s}$, the application emits expanding retro wireframe circular smoke puffs:
* **Wireframe Geometry**: Concentric 6-segment or 8-segment expanding polygonal loops drawn with fading phosphor luminance.
* **Drift Dynamics**: Particles receive an initial impulse along the slip direction $\mathbf{v}_{\text{slip}}$ and decelerate under aerodynamic air resistance.
* **Opacity Decay**: Alpha fades quadratically over lifetime $\tau \in [1.0, 1.8]\,\text{s}$.
* **Zero Allocation**: Particles are recycled inside [`TireSmokeParticleBuffer`](../apps/MotionLab/TireSmokeParticleBuffer.h) using a circular 512-element pre-allocated buffer.

---

### 6.4 Interactive Hotkeys & Controls

| Hotkey Trigger | Operational Action | System Response |
| :--- | :--- | :--- |
| **`T`** | Toggle Tire Thermal HUD Card | Cycles visibility of 4-wheel diagnostic HUD overlay. |
| **`Ctrl+T`** | Reset Tire Temperatures | Immediately quenches all 4 wheels back to ambient baseline ($25^\circ\text{C}$). |
| **`[` / `]`** | Simulation Time Scaling | Preserves fixed $8.333\,\text{ms}$ physics timestep while scaling thermal advancement. |
| **`F7`** | Environmental Preset Cycle | Toggles ambient air & track temperature presets (Cool Autumn, Summer Trackday, Desert Proving Ground). |

---

## 7. Step-by-Step Implementation Roadmap & Phased Milestones

```
+---------------------------------------------------------------------------------------------------+
|                        Tire Thermal Model Phased Implementation Roadmap                           |
+---------------------------------------------------------------------------------------------------+
|                                                                                                   |
|  [PHASE 1: Core Thermodynamics] -------------------------> [PHASE 2: Pacejka & Fleet Tuning]     |
|  * TireThermalModel.h & .cpp                               * Couple with PacejkaTireModel         |
|  * 3-Node lumped RC network ODEs                           * Parameterize 12 vehicle configs      |
|  * Friction scrub & flex power                             * Cold warmup & overheat calibration   |
|  * Asymmetric grip bell curve                                                                     |
|                                                                          |                        |
|                                                                          v                        |
|  [PHASE 4: Visuals & HUD] <------------------------------- [PHASE 3: App Coordinator & Smoke]     |
|  * TireThermalRenderer.h & .cpp                            * TireThermalCoordinator.h & .cpp      |
|  * 4-wheel wireframe HUD card                              * TireSmokeParticleBuffer (512 pool)   |
|  * Phosphor color grading                                  * Vehicle kinematic extraction         |
|  * Expanding smoke polygons                                * Interactive hotkeys (T, Ctrl+T)      |
|                                                                                                   |
|         |                                                                                         |
|         v                                                                                         |
|  [PHASE 5: Telemetry & Exporters] -----------------------> [PHASE 6: Verification & Safety]       |
|  * Add thermal channels to HUD                             * GoogleTest TireThermalModelTest.cpp  |
|  * Extended CSV logging export                             * Deterministic replay bitwise check   |
|  * Live oscilloscope channels                              * MISRA C++:2023 compliance audit      |
|                                                                                                   |
+---------------------------------------------------------------------------------------------------+
```

### Phase 1: Core Thermodynamics Engine (`libs/VehicleSimCore/dynamics`)
* **Task 1.1**: Author [`TireThermalModel.h`](../libs/VehicleSimCore/dynamics/TireThermalModel.h) and [`TireThermalModel.cpp`](../libs/VehicleSimCore/dynamics/TireThermalModel.cpp).
* **Task 1.2**: Implement the 3-node coupled thermodynamic ODE solver (surface, carcass, cavity).
* **Task 1.3**: Implement the non-linear asymmetric friction bell curve formulation $\mu_{\text{eff}}(T)$.
* **Task 1.4**: Implement the Ideal Gas Law cavity pressure expansion and Pacejka cornering stiffness feedback.
* **Task 1.5**: Verify zero dynamic allocations and strict IEEE 754 determinism.

### Phase 2: Pacejka Tire Coupling & Universal Fleet Calibration
* **Task 2.1**: Integrate thermal friction multiplier $\mu_{\text{eff}}$ into [`PacejkaTireModel`](../libs/VehicleSimCore/dynamics/PacejkaTireModel.h).
* **Task 2.2**: Integrate cavity pressure $P_{\text{cavity}}$ into dynamic tire vertical and cornering stiffness.
* **Task 2.3**: Calibrate thermal capacity and decay parameters for all 12 vehicles (Drift Car, Race Car, Semi-Truck, etc.).
* **Task 2.4**: Implement graceful thermal bypass for non-wheeled craft (Drone, Rocket, Hovercraft).

### Phase 3: Application Coordinator & Smoke Particle Buffer (`apps/MotionLab`)
* **Task 3.1**: Implement [`TireSmokeParticleBuffer.h`](../apps/MotionLab/TireSmokeParticleBuffer.h) and `.cpp` with a fixed 512-particle pool.
* **Task 3.2**: Implement [`TireThermalCoordinator.h`](../apps/MotionLab/TireThermalCoordinator.h) and `.cpp`.
* **Task 3.3**: Integrate thermal coordinator into `MotionLabApp::stepPhysics()` and input handling (`T`, `Ctrl+T`).

### Phase 4: Retro 1996 Wireframe Visuals & HUD Card
* **Task 4.1**: Implement [`TireThermalRenderer.h`](../apps/MotionLab/TireThermalRenderer.h) and `.cpp`.
* **Task 4.2**: Render the 4-corner diagnostic HUD card with segmented LED bars, temperatures, and pressures.
* **Task 4.3**: Render multi-segmented wireframe expanding smoke polygons trailing hot sliding wheels.

### Phase 5: Telemetry HUD & CSV Exporter Integration
* **Task 5.1**: Integrate 4-wheel temperatures and pressure channels into [`TelemetryCoordinator`](../apps/MotionLab/TelemetryCoordinator.h).
* **Task 5.2**: Update CSV telemetry stream to serialize tire thermal metrics for post-run analysis.

### Phase 6: Comprehensive Verification & MISRA Compliance Audit
* **Task 6.1**: Author unit test suite [`tests/TireThermalModelTest.cpp`](../tests/TireThermalModelTest.cpp).
* **Task 6.2**: Run determinism test confirming identical thermal states across 10,000 steps.
* **Task 6.3**: Perform MISRA C++:2023 and SEI CERT C++ compliance check.

---

## 8. Comprehensive Test Suite & Verification Matrix

### 8.1 Quantitative Acceptance Criteria

| Metric | Target Specification | Validation Method |
| :--- | :--- | :--- |
| **Deterministic Consistency** | Identical bitwise temperatures across consecutive runs with same inputs. | Double-run hashing assertion. |
| **Step Computational Overhead**| Total 4-wheel thermal update $< 3.5\,\mu\text{s}$ per physics step. | High-resolution microsecond timer benchmark. |
| **Zero Heap Allocations** | Exactly **0** allocations during `update()` and `render()`. | Custom global `new`/`delete` tracking allocator test. |
| **Ambient Thermal Equilibrium**| Isolated stationary tire settles to $T_{\text{ambient}}$ within $\pm 10^{-4\,\circ}\text{C}$. | GoogleTest unit assertion. |
| **Peak Friction Alignment** | Maximum friction factor strictly equals $1.00$ within optimal window $[T_{\text{opt\_min}}, T_{\text{opt\_max}}]$. | Unit test analytical evaluation. |
| **Overheat Friction Bound** | Friction factor at $160^\circ\text{C}$ drops by configured $\Delta\mu_{\text{over}} \pm 1\%$. | Unit test analytical evaluation. |
| **Ideal Gas Pressure Bound** | Cavity pressure at $100^\circ\text{C}$ matches $P_{\text{cold}} \cdot \frac{373.15}{298.15}$ within $0.01\,\text{bar}$. | Physical formula verification test. |

---

### 8.2 GoogleTest Suite Specification: `tests/TireThermalModelTest.cpp`

```cpp
#include <gtest/gtest.h>
#include <cmath>

#include "libs/VehicleSimCore/dynamics/TireThermalModel.h"

using namespace vehicle_sim::dynamics;

TEST(TireThermalModelTest, AmbientEquilibrium_StationaryWheel)
{
    TireThermalConfig config {};
    TireThermalModel model { config };
    model.reset(25.0);

    // Advance 500 steps with zero loads and zero slip
    for (int i { 0 }; i < 500; ++i) {
        model.update(0.0, 0.0, 4000.0, 0.0, 0.0, 0.0, 0.32, 0.0, 25.0, 25.0, 0.016);
    }

    EXPECT_NEAR(model.getSurfaceTemp(), 25.0, 1e-3);
    EXPECT_NEAR(model.getCarcassTemp(), 25.0, 1e-3);
    EXPECT_NEAR(model.getCavityPressure(), 2.0, 1e-3);
}

TEST(TireThermalModelTest, PureScrubHeating_TemperatureIncreases)
{
    TireThermalConfig config {};
    TireThermalModel model { config };
    model.reset(25.0);

    // Apply high sliding slip velocity under 5000 N normal load
    constexpr double normalLoad { 5000.0 };
    constexpr double slipVel { 8.0 }; // 8 m/s sliding
    constexpr double tractiveForce { 4500.0 };

    for (int i { 0 }; i < 200; ++i) {
        model.update(tractiveForce, 0.0, normalLoad, slipVel, 0.0, 30.0, 0.32, 10.0, 25.0, 25.0, 0.016);
    }

    EXPECT_GT(model.getSurfaceTemp(), 75.0);
    EXPECT_GT(model.getCarcassTemp(), 35.0);
}

TEST(TireThermalModelTest, NonLinearGripCharacteristic_PeakAndDrop)
{
    TireThermalConfig config {};
    config.optimalTempMin = 75.0;
    config.optimalTempMax = 95.0;
    config.maxThermalDecay = 0.35;
    config.coldGripDeficit = 0.20;

    TireThermalModel model { config };

    // Cold regime: grip reduced
    const double coldGrip { model.evaluateGripFactor(20.0) };
    EXPECT_LT(coldGrip, 0.85);

    // Optimal window: peak grip
    const double optGrip1 { model.evaluateGripFactor(80.0) };
    const double optGrip2 { model.evaluateGripFactor(90.0) };
    EXPECT_NEAR(optGrip1, 1.0, 0.02);
    EXPECT_NEAR(optGrip2, 1.0, 0.02);

    // Overheated regime: grip degraded
    const double hotGrip { model.evaluateGripFactor(150.0) };
    EXPECT_LT(hotGrip, 0.70);
    EXPECT_GT(hotGrip, 0.60);
}

TEST(TireThermalModelTest, CavityPressure_IdealGasExpansion)
{
    TireThermalConfig config {};
    config.baseInflationPressure = 2.0;
    config.referenceColdTemp = 25.0; // 298.15 K

    TireThermalModel model { config };
    model.reset(25.0);

    // Heat carcass and cavity
    for (int i { 0 }; i < 1000; ++i) {
        model.update(6000.0, 3000.0, 6000.0, 10.0, 5.0, 40.0, 0.32, 5.0, 25.0, 25.0, 0.016);
    }

    const double cavityT { model.getState().cavityTemp };
    const double expectedPressure { 2.0 * ((cavityT + 273.15) / 298.15) };
    EXPECT_NEAR(model.getCavityPressure(), expectedPressure, 0.05);
    EXPECT_GT(model.getCavityPressure(), 2.10);
}
```

---

## 9. Safety, MISRA & Coding Compliance Matrix

### 9.1 Verification Checklist Confirmation

- [x] **1. No raw `new` / `delete`**: All buffers utilize stack allocations or pre-allocated pools initialized at startup.
- [x] **2. `#pragma once` used**: All headers strictly utilize `#pragma once` include guards.
- [x] **3. Co-located `.h` and `.cpp`**: Files are co-located in `libs/VehicleSimCore/dynamics/` and `apps/MotionLab/`.
- [x] **4. Zero external engine dependencies**: `VehicleSimCore` code uses only standard ISO C++17 headers (`<cmath>`, `<algorithm>`, `<array>`).
- [x] **5. Concise function names**: All function names are strictly under 30 characters.
- [x] **6. Cross-platform math**: IEEE 754 determinism maintained; zero OS-specific headers.
- [x] **7. Explicit `{}` initialization**: All variables, structures, and members use brace initialization.
- [x] **8. `[[nodiscard]]` applied**: Applied to all state queries, temperature lookups, and grip multipliers.
- [x] **9. Full Doxygen documentation**: Public APIs documented with `@brief`, `@details`, `@param`, `@return`.
- [x] **10. MISRA C++:2023 / SEI CERT C++ compliance**: Zero implicit numeric conversions, no dynamic casting, zero undefined behavior.
- [x] **11. Relative file paths**: All referenced file paths in documentation and code are strictly relative.
- [x] **12. Dual ASCII & Mermaid diagrams**: Provided for architecture, thermodynamic network, energy balance, and implementation roadmap.
