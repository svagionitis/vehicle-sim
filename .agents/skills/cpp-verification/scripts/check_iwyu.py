#!/usr/bin/env python3
"""
Include-What-You-Use (IWYU) Verification Script.

Enforces:
1. Header Self-Containment: Every header must directly include all headers it depends on.
2. Direct Standard Library Inclusions: Any file using standard library symbols/templates
   (e.g., std::vector, std::optional, std::unique_ptr, std::clamp, std::pair, std::uint32_t)
   must directly #include their canonical header instead of relying on transitive includes.
3. Optional Compiler IWYU: If --tool is passed, invokes `iwyu_tool -p build` using compilation database.
"""

import sys
import os
import re
import subprocess
import argparse

STD_IWYU_RULES = [
    ("cstdint", re.compile(r"\bstd::u?int(8|16|32|64)_t\b"), re.compile(r"#\s*include\s*[<\"_]c?stdint(\.h)?[>\"]")),
    ("cstddef", re.compile(r"\bstd::(size_t|ptrdiff_t|byte)\b"), re.compile(r"#\s*include\s*[<\"_]c?stddef(\.h)?[>\"]")),
    ("vector", re.compile(r"\bstd::vector<"), re.compile(r"#\s*include\s*[<\"_]vector[>\"]")),
    ("string", re.compile(r"\bstd::(string|to_string|getline)\b"), re.compile(r"#\s*include\s*[<\"_]string[>\"]")),
    ("string_view", re.compile(r"\bstd::string_view\b"), re.compile(r"#\s*include\s*[<\"_]string_view[>\"]")),
    ("array", re.compile(r"\bstd::array<"), re.compile(r"#\s*include\s*[<\"_]array[>\"]")),
    ("memory", re.compile(r"\bstd::(unique_ptr|shared_ptr|make_unique|make_shared|weak_ptr)\b"), re.compile(r"#\s*include\s*[<\"_]memory[>\"]")),
    ("optional", re.compile(r"\bstd::(optional<|nullopt\b)"), re.compile(r"#\s*include\s*[<\"_]optional[>\"]")),
    ("variant", re.compile(r"\bstd::(variant<|holds_alternative|get_if)"), re.compile(r"#\s*include\s*[<\"_]variant[>\"]")),
    ("utility", re.compile(r"\bstd::(pair<|move\(|forward<|make_pair\()"), re.compile(r"#\s*include\s*[<\"_]utility[>\"]")),
    ("algorithm", re.compile(r"\bstd::(clamp|sort|copy|fill|transform|find|find_if|count|count_if|min_element|max_element)\b"), re.compile(r"#\s*include\s*[<\"_]algorithm[>\"]")),
    ("cmath", re.compile(r"\bstd::(sin|cos|tan|sqrt|atan2|floor|ceil|round|fmod|exp|log)\b"), re.compile(r"#\s*include\s*[<\"_]cmath[>\"]")),
    ("chrono", re.compile(r"\bstd::chrono::"), re.compile(r"#\s*include\s*[<\"_]chrono[>\"]")),
    ("atomic", re.compile(r"\bstd::atomic<"), re.compile(r"#\s*include\s*[<\"_]atomic[>\"]")),
    ("mutex", re.compile(r"\bstd::(mutex|lock_guard|unique_lock)\b"), re.compile(r"#\s*include\s*[<\"_]mutex[>\"]")),
    ("limits", re.compile(r"\bstd::numeric_limits<"), re.compile(r"#\s*include\s*[<\"_]limits[>\"]")),
]

def check_file_iwyu(path, is_header_only_mode=False):
    issues = []
    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as fp:
            content = fp.read()
    except Exception as e:
        return [f"{path}: Cannot read file: {e}"]

    # Strip comments to avoid false positives in documentation
    clean_code = re.sub(r"//.*|/\*.*?\*/", "", content)

    for header, sym_pat, inc_pat in STD_IWYU_RULES:
        if sym_pat.search(clean_code) and not inc_pat.search(content):
            issues.append(f"{path}: Missing '#include <{header}>' for used standard library symbol(s).")

    return issues

def check_header_compilation(header_path):
    """Verifies that a header compiles standalone in clean C++17."""
    cmd = [
        "g++", "-std=c++17", "-I.", "-fsyntax-only", "-x", "c++",
        "-Werror=missing-include-dirs", header_path
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        err = res.stderr.strip().splitlines()
        first_err = err[0] if err else "Unknown compilation error"
        return [f"{header_path}: Header is not self-contained: {first_err}"]
    return []

def run_iwyu_tool(build_dir, sources):
    tool_path = "/usr/bin/iwyu_tool"
    if not os.path.exists(tool_path):
        tool_path = "iwyu_tool"
    cmd = [tool_path, "-p", build_dir] + sources
    print(f"Running IWYU compiler tool: {' '.join(cmd[:5])}...")
    res = subprocess.run(cmd)
    return res.returncode

def main():
    parser = argparse.ArgumentParser(description="Include-What-You-Use (IWYU) Verification Tool")
    parser.add_argument("paths", nargs="*", help="Files or directories to scan (defaults to libs, apps, tests)")
    parser.add_argument("--headers-only", action="store_true", help="Only verify header files for strict self-containment")
    parser.add_argument("--check-compilation", action="store_true", help="Compile each header standalone with g++ -fsyntax-only")
    parser.add_argument("--tool", action="store_true", help="Run external iwyu_tool with compile_commands.json")
    parser.add_argument("--build-dir", default="build", help="Path to cmake build directory (default: build)")
    args = parser.parse_args()

    if args.tool:
        return run_iwyu_tool(args.build_dir, args.paths)

    files = []
    targets = args.paths if args.paths else ["libs", "apps", "tests"]
    for t in targets:
        if os.path.isfile(t):
            files.append(t)
        elif os.path.isdir(t):
            for root, _, filenames in os.walk(t):
                for f in filenames:
                    if f.endswith((".h", ".hpp", ".cpp")):
                        files.append(os.path.join(root, f))

    if not files:
        print("No files found to verify.")
        sys.exit(0)

    all_issues = []
    for path in sorted(files):
        is_h = path.endswith((".h", ".hpp"))
        if args.headers_only and not is_h:
            continue
        issues = check_file_iwyu(path, is_header_only_mode=args.headers_only)
        all_issues.extend(issues)

        if args.check_compilation and is_h:
            comp_issues = check_header_compilation(path)
            all_issues.extend(comp_issues)

    if all_issues:
        print(f"FAILED: Found {len(all_issues)} IWYU issue(s):")
        for iss in all_issues:
            print(f"  - {iss}")
        sys.exit(1)
    else:
        print(f"PASSED: Scanned {len(files)} file(s). All checked headers and source files adhere to IWYU rules.")
        sys.exit(0)

if __name__ == "__main__":
    main()
