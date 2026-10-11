# Git Commit Instructions

When writing a commit message, follow these strict repository rules:

## 1. Subject Line Format
- **Format:** `<Scope>: <verb in imperative> <summary>`
  - Scope is PascalCase/CamelCase identifying the affected component:
    - `Core`: `libs/VehicleSimCore` general updates
    - `Math`: Vector and transform math utilities (`Vec2`, `Mat2`, `Transform2D`)
    - `Physics`: Rigid body, Symplectic Euler integrator, force accumulators
    - `Dynamics`: Vehicle dynamics models (`ArcadeCar`, `MonsterCar`, `SideViewCar`, `Rocket`)
    - `Collision`: Barrier and contact impulse solvers
    - `Surface`: Surface grip and track property models
    - `Tests`: Unit tests and physics verification benchmarks
    - `Docs`: Architecture, mathematical specs, and API documentation
    - `CMake`: Build system, hardening flags, and sanitizers
  - Use `/` for multi-component changes (e.g., `Core/Math:`, `Dynamics/Rocket:`).
  - Use brace expansion for sibling units (e.g., `RigidBody2D{,.cpp}:`).
- **Verb:** Use lowercase imperative mood immediately after the colon (`fix`, `add`, `implement`, `refactor`, `align`, `remove`, `configure`).
- **Punctuation:** Do not end the subject line with a period.
- **Length:** Target <= 50 characters when possible; 72 characters is the absolute hard limit (especially with multi-component scopes).

## 2. Body Structure
- **Blank Line:** Separate subject and body with exactly one blank line.
- **Wrapping:** Manually hard-wrap all body lines at 72 characters.
- **Summary Paragraph:** Start with a brief overview explaining the problem, context, and rationale (*what* and *why*, not raw code diffs).
- **Component Breakdown:** Group technical details by component bullet points:
  ```text
  - libs/VehicleSimCore/ComponentName:
    - Detail specific design shifts or API adjustments.
    - Explain non-obvious fixes or invariants maintained.
  ```

