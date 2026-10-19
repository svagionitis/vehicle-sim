---
name: cpp-verification
description: >-
  Use this skill before outputting C++17 code snippets to verify safety, style, and compliance against the project checklist.
---

# C++17 Verification Checklist

Before presenting any C++17 code snippet, internally verify it against this checklist:

1. **RAII / Memory:** Did I use any raw `new` or `delete`? (If yes, rewrite using RAII/Smart Pointers).
2. **Initialization:** Are all variables explicitly initialized using `{}` where possible?
3. **Safety Attributes:** Did I apply `[[nodiscard]]` to functions returning safety-critical data?
4. **Documentation:** Are all public APIs documented with full Doxygen templates?
5. **Standards Compliance:** Does this code violate any MISRA/AUTOSAR rules (e.g., dynamic casting, implicit conversions)?
6. **TDD:** Did I write a failing test first if fixing a bug?
7. **Header Guards:** Did I use `#pragma once` at the top of headers instead of legacy `#ifndef` / `#define` guards?
8. **Fixed-Width Integers (<cstdint>):** Are fixed-width types used (e.g., `std::uint32_t`, `std::int32_t`) instead of platform-dependent basic integer types? Is `#include <cstdint>` directly included in every header and translation unit using them (no leaky transitive includes)? Are all types strictly qualified in `std::`?
9. **Include-What-You-Use (IWYU):** Does every header and implementation file directly `#include` all symbols/templates it consumes (e.g., `<vector>`, `<string>`, `<optional>`, `<cstdint>`, `<utility>`)? Are headers 100% self-contained?

## Automated Verification Scripts
Run the automated `<cstdint>` compliance verification script:
```bash
python3 .agents/skills/cpp-verification/scripts/check_cstdint.py [paths...]
```

Run the automated Include-What-You-Use (IWYU) verification script:
```bash
python3 .agents/skills/cpp-verification/scripts/check_iwyu.py [paths...]
```
