# Architecture & Error Handling

## Project Architecture & Layer Boundaries
- **Core Architecture:**
  - `libs/VehicleSimCore`: Standalone, pure C++17 library. Zero external physics/game engine dependencies. Must be usable by any game engine, rendering backend, or simulation harness (Raylib, SDL, SFML, Unreal, Unity native plugin, or CLI).
  - Modular subcomponents:
    - `math`: Fixed-width, header-focused 2D linear algebra (`Vec2`, `Mat2`, `Transform2D`).
    - `physics`: 2D rigid body dynamics, numerical integration (Symplectic Euler), impulse contact solver.
    - `dynamics`: Swappable vehicle dynamics coordinators (`ArcadeCarDynamics`, `MonsterCarDynamics`, `SideViewCarDynamics`, `RocketDynamics`).
    - `surface`: Friction maps and environmental modifiers (dirt, tarmac, mud, ice, oil slick).
- **Co-located File Organization:** Headers (`.h`) and implementations (`.cpp`) must reside in the same directory. Never create separate `include/` and `src/` directories.
- **Modular CMake & Security Hardening:** Core and test modules apply `apply_compiler_flags` from `cmake/CompilerFlags.cmake` (`-Werror`, `-Wconversion`, `-fstack-protector-strong`, sanitizers).
- **Header Inclusion:** Always use `#pragma once` across all headers. Never use `#ifndef / #define` include guards.
- **Cross-Platform Portability (Linux & Windows):**
  - All core logic, vector math, and physics engines must compile cleanly and execute deterministically across **Linux** (GCC >= 9, Clang >= 10) and **Windows** (MSVC >= 19.28 / Visual Studio 2019+, Clang-CL).
  - Pure standard C++17: No platform-specific headers (`<windows.h>`, `<unistd.h>`) or platform APIs allowed in `VehicleSimCore`.
  - Floating-point determinism: Rely exclusively on standard `<cmath>` functions (`std::atan2`, `std::hypot`, `std::sin`, `std::cos`, `std::clamp`) with IEEE 754 compliance.
  - Strict compiler warning parity: Must build with zero warnings under `-Wall -Wextra -pedantic -Wconversion -Werror` (Linux/Clang) and `/W4 /WX /permissive-` (MSVC).

## Vehicle Simulation Engine Design
- **Coordinator Pattern (Pat Kerr Model):**
  - Rigid body kinematics (`RigidBody2D`: mass, moment of inertia, position, linear velocity, orientation, angular velocity) remain persistent.
  - Dynamics behaviors (`IVehicleDynamics`) attach to the body and translate driver/pilot inputs into accumulated forces and torques.
  - Modes can be swapped at runtime (e.g. Car -> Rocket -> Platformer) without resetting position or linear/angular momentum.
- **Dual-Model Tire Simulation:**
  - *Arcade Model (Pat Kerr)*: Decomposes point velocities at wheels into longitudinal and lateral components; applies velocity-proportional damping resistance; handbrake scales rear lateral grip and increases rear drag. Ideal for high-speed drift games like *Offroad*.
  - *Dynamic Model (Marco Monster)*: Computes dynamic longitudinal weight transfer ($W_f, W_r$), wheel slip angles ($\alpha = \arctan(v_{lat}/v_{long})$), cornering stiffness forces capped by the friction circle ($\mu W$), engine torque curves, and aero drag ($F_{drag} \propto v^2$).
- **Multi-Movement Physics Support:**
  - *Top-Down 2D Horizontal (Offroad)*: 4-wheel independent tire contacts, surface friction coefficients, bump/airborne jump states, nitro boost impulse.
  - *Side-View 2D Platform Driving (City Connection)*: Gravity integration, platform contact normal impulses, jump impulse mechanics, instant 180° turnaround torque, oil slick loss-of-traction state.
  - *2D Space Movement & Rockets (Asteroids / Thrust / Future Rockets)*: Main thruster vectoring with gimbal angle, offset thrust point generating angular torque, RCS attitude control thrusters, mass depletion over time with variable moment of inertia.

## Abstraction & Layer Boundaries
- **Program to Levels of Abstraction:** Lower-level mechanics (numerical integration, vector math, impulse equations) must be encapsulated in `physics` and `math`. High-level game interaction exposes semantic inputs (`throttle`, `steer`, `handbrake`, `thrust`, `gimbalAngle`, `jump`).
- **Strict Layer Boundary Hierarchy:** High-level vehicle coordinators interact with `RigidBody2D` only via public force/torque accumulation or contact impulse APIs.
- **Member Visibility:** Treat member visibility changes as a breaking design shift. Keep all fields and functions private unless external access is strictly required by the design. Prompt the user for explicit approval before changing any access modifier from private to internal or public.

## Error Handling & Safety
- **No Exceptions in Critical Paths:** Simulation integration and force calculations must not throw exceptions. Report invalid states via `bool`, `std::optional`, or enum status codes.
- **`noexcept` Specification:** Mark all functions that are guaranteed not to throw as `noexcept` (especially destructors, move constructors, move assignment operators, and pure getters/parsers/math operations).

## OOP & Expressions
- **Virtual Destructors:** Every base class with virtual functions (e.g. `IVehicleDynamics`) must explicitly declare a `virtual` destructor. Always use the `override` specifier for overridden virtual functions without repeating the `virtual` keyword.
- **No C-Style Casts:** Use `static_cast`, `const_cast`, or `reinterpret_cast` (only when absolutely necessary). Never use `(Type)value`.
- **No Side Effects in Evaluated Contexts:** Avoid pre/post-increment expressions mixed inside complex expressions (e.g., `array[i++] = ++j;` is banned). Keep statements atomic.


