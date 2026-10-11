---
trigger: always_on
---

# Verification Checklist (Run Before Outputting Code)

Before presenting any C++17 code snippet or committing changes, internally verify against this checklist:
1. Did I use any raw `new` or `delete`? (If yes, rewrite using RAII/Smart Pointers).
2. Are headers using `#pragma once`? (Never use `#ifndef / #define` include guards).
3. Are header (`.h`) and implementation (`.cpp`) files co-located in the same directory?
4. Does `VehicleSimCore` maintain **zero external engine dependencies** (pure C++17)?
5. Are all function names concise (strictly less than 30 characters)?
6. Does math and physics code support both **Linux** (GCC/Clang) and **Windows** (MSVC/Clang), with zero OS-specific headers and IEEE 754 determinism?
7. Are all variables explicitly initialized using `{}` where possible?
8. Did I apply `[[nodiscard]]` to functions returning status or safety-critical data?
9. Are all public APIs documented with full Doxygen templates (`/// @brief`, `@details`, `@param`, `@return`)?
10. Does this code violate any MISRA/AUTOSAR rules (e.g., dynamic casting, implicit conversions)?
11. Did I write a failing test first if fixing a bug?
12. Does code compile with zero warnings under `-Wall -Wextra -pedantic` (`-Werror`) and `/W4` on MSVC, with hardening flags applied?
13. All the paths of the filenames in the docs should be relative.
14. If there are diagrams involved, add both ASCII and mermaid diagrams.

