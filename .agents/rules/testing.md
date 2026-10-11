# Testing & Verification

## Test-Driven Bug Fixing
When a bug is being fixed:
1. Do not write the fix right away.
2. First, write the test that reproduces the bug.
3. Observe it failing (Red).
4. Write the fix.
5. Observe the test passing (Green).

## Repository Testing Conventions
- **Automated CTest Suite:** Run tests via `ctest --test-dir build --output-on-failure`. All tests must pass 100%.
- **Analytical Physics Validation:** Test straight-line acceleration against terminal velocity equilibrium ($F_{\text{traction}} = F_{\text{drag}} + F_{\text{rr}}$), weight transfer balance, circular skidpad steady-state radius, and Symplectic Euler energy conservation.
- **State-Preserving Mode Switches:** Verify that swapping between dynamics models (`ArcadeCar`, `MonsterCar`, `SideViewCar`, `Rocket`) preserves body momentum and coordinate integrity.
- **Deterministic Numerical Tests:** Physics integration over fixed $\Delta t$ must yield bit-identical or epsilon-bounded results across repeated runs and platforms.

