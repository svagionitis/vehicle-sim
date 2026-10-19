---
trigger: always_on
---

# Include-What-You-Use (IWYU) Compliance

To maintain robust, modular code and prevent compilation regressions across GCC, Clang, and MSVC:

## 1. Header Self-Containment
- Every header (`.h` / `.hpp`) must directly `#include` all header files declaring types, functions, or templates referenced in its declarations or inline definitions.
- Never rely on another header to transitively pull in Standard Library components (e.g., `<vector>`, `<string>`, `<memory>`, `<optional>`, `<cstdint>`, `<cstddef>`, `<utility>`, `<cmath>`, `<algorithm>`).
- Every header must compile standalone without pre-existing context:
  ```bash
  g++ -std=c++17 -I. -fsyntax-only -x c++ path/to/header.h
  ```

## 2. Direct Inclusions in Translation Units (`.cpp`)
- Implementation files must directly `#include` every header providing symbols used within their translation units.
- Do not rely on indirect dependencies through other project headers unless that header's documented public interface explicitly re-exports or is designed to provide those types.

## 3. Forward Declarations
- Use forward declarations in header files where only pointers or reference parameters/members (`const T&`, `T*`) are used, reducing header dependency fan-out and compilation times.
- Include the complete header in the `.cpp` file where object layouts, methods, or member variables are accessed.

## 4. Verification Check
- Verify IWYU compliance before committing or presenting code changes:
  ```bash
  python3 .agents/skills/cpp-verification/scripts/check_iwyu.py [paths...]
  ```
- Or run compiler-level AST IWYU with the compilation database:
  ```bash
  iwyu_tool -p build [paths...]
  ```
