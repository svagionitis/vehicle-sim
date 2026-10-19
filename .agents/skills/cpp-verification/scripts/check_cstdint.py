#!/usr/bin/env python3
"""
Verification script for <cstdint> compliance.
Checks:
1. Include-What-You-Use: Any file using fixed-width integer types must directly #include <cstdint>.
2. Namespace Qualification: Fixed-width integer types must be prefixed with std:: (no bare uint32_t).
"""

import sys
import os
import re

CSTDINT_TYPES = re.compile(r"\b(std::)?(u?int(8|16|32|64)_t|intptr_t|uintptr_t)\b")
CSTDINT_INC = re.compile(r"#\s*include\s*[<\"_]c?stdint(\.h)?[>\"]")
UNPREFIXED = re.compile(r"(?<!std::)\b(u?int(8|16|32|64)_t|intptr_t|uintptr_t)\b")

def check_file(path):
    issues = []
    with open(path, "r", encoding="utf-8", errors="ignore") as fp:
        lines = fp.readlines()

    full_content = "".join(lines)
    uses_fixed = CSTDINT_TYPES.search(full_content)
    has_inc = CSTDINT_INC.search(full_content)

    if uses_fixed and not has_inc:
        issues.append(f"{path}: Missing '#include <cstdint>' while using fixed-width integer types.")

    for idx, line in enumerate(lines):
        line_clean = re.sub(r"//.*|/\*.*?\*/", "", line)
        if UNPREFIXED.search(line_clean):
            matches = UNPREFIXED.findall(line_clean)
            types_found = [m[0] for m in matches]
            issues.append(f"{path}:{idx + 1}: Unqualified fixed-width type(s) {types_found} used without 'std::' prefix.")

    return issues

def main():
    targets = sys.argv[1:]
    files = []

    if not targets:
        search_dirs = ["libs", "apps", "tests"]
        for sdir in search_dirs:
            if not os.path.exists(sdir):
                continue
            for root, _, filenames in os.walk(sdir):
                for f in filenames:
                    if f.endswith((".h", ".hpp", ".cpp")):
                        files.append(os.path.join(root, f))
    else:
        for target in targets:
            if os.path.isfile(target):
                files.append(target)
            elif os.path.isdir(target):
                for root, _, filenames in os.walk(target):
                    for f in filenames:
                        if f.endswith((".h", ".hpp", ".cpp")):
                            files.append(os.path.join(root, f))

    all_issues = []
    for f in sorted(files):
        issues = check_file(f)
        all_issues.extend(issues)

    if all_issues:
        print(f"FAILED: Found {len(all_issues)} <cstdint> compliance issue(s):")
        for issue in all_issues:
            print(f"  - {issue}")
        sys.exit(1)
    else:
        print(f"PASSED: Scanned {len(files)} files. Zero <cstdint> compliance issues found.")
        sys.exit(0)

if __name__ == "__main__":
    main()
