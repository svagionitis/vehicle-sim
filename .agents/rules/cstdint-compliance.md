---
trigger: always_on
---

# Fixed-Width Integer Types & `<cstdint>` Compliance

To satisfy **MISRA C++:2023 (Rule 6.1.1)**, **SEI CERT C++**, and cross-platform determinism across Linux and Windows:

## 1. Include-What-You-Use (IWYU)
- Every header (`.h`) and implementation (`.cpp`) file that references fixed-width integer types must directly include `<cstdint>`.
- Never rely on transitive includes from other headers (e.g., `<vector>`, `<string>`, or `<cmath>`).

## 2. Namespace Qualification
- All fixed-width integer types must be explicitly qualified with `std::` (e.g., `std::uint8_t`, `std::uint16_t`, `std::uint32_t`, `std::uint64_t`, `std::int8_t`, `std::int16_t`, `std::int32_t`, `std::int64_t`).
- Bare C-style types (`uint32_t`, `int32_t`, etc.) in the global namespace are strictly forbidden.

## 3. Basic Integer Type Restrictions
- Do not use platform-dependent basic types (`int`, `unsigned int`, `long`, `unsigned long`, `short`, `unsigned short`, `unsigned char`) for simulation state, counters, discrete physics parameters, audio DSP, bitmasks, or enums.
- Use explicit `<cstdint>` types (`std::int32_t`, `std::uint32_t`, `std::uint8_t`, etc.).
- Permitted exceptions:
  - `int main(int argc, char* argv[])`
  - `std::size_t` from `<cstddef>` for container indices and buffer sizes.
  - `char` for text characters and string operations.
  - `bool` for boolean operations.
  - SDL2 / external library C API parameters requiring primitive `int` at the boundary interface.

## 4. Verification
- Verify compliance prior to presenting code using the automated check:
  ```bash
  python3 .agents/skills/cpp-verification/scripts/check_cstdint.py [paths...]
  ```
