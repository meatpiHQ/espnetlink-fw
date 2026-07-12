#!/usr/bin/env python3
"""Run the host-test CI locally, without pushing.

Mirrors .github/workflows/host-tests.yml: the suite list is parsed from
the workflow file itself (so this script can't drift from CI), and every
suite is built for the IDF `linux` target and executed inside the same
`espressif/idf:v6.0.2` container CI uses. A suite passes when its .elf
exits 0.

Requires: Docker with the espressif/idf image (pulled on first use).

Usage:
    python tools/run_host_tests.py             # all suites
    python tools/run_host_tests.py gps vpn     # only suites matching a substring

All suites run in ONE container invocation (export.sh is slow); each
suite's full build/run log lands in build/host_tests/<component>.log and
the failing tail is echoed inline. Exit code = number of failed suites.
"""
import argparse
import pathlib
import re
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
WORKFLOW = REPO / ".github" / "workflows" / "host-tests.yml"
IMAGE = "espressif/idf:v6.0.2"
LOG_DIR = "build/host_tests"  # container-side, under the repo mount


def suites_from_workflow():
    text = WORKFLOW.read_text(encoding="utf-8")
    suites = re.findall(r"^\s*-\s+(components/\S+/host_test)\s*$", text, re.M)
    if not suites:
        sys.exit(f"no suites found in {WORKFLOW}")
    return suites


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("filter", nargs="*",
                    help="only run suites whose path contains any of these substrings")
    args = ap.parse_args()

    suites = suites_from_workflow()
    if args.filter:
        suites = [s for s in suites if any(f in s for f in args.filter)]
        if not suites:
            sys.exit(f"filter {args.filter} matched no suites")

    print(f"{len(suites)} suite(s) in {IMAGE}:")
    for s in suites:
        print(f"  {s}")

    # One bash script for the whole matrix: export once, loop, keep going
    # on failure, report a summary line per suite that this side parses.
    inner = f"""
set -u
. "$IDF_PATH/export.sh" >/dev/null 2>&1
git config --global --add safe.directory /work >/dev/null 2>&1 || true
mkdir -p {LOG_DIR}
fail=0
for suite in {' '.join(suites)}; do
    name=$(basename $(dirname "$suite"))
    log={LOG_DIR}/$name.log
    ok=0
    # two attempts: the Windows-mounted volume throws transient
    # "Permission denied" file locks at cmake/ninja now and then
    for attempt in 1 2; do
        (
            cd "$suite" &&
            idf.py --preview set-target linux &&
            idf.py build &&
            elf=$(ls build/*.elf) &&
            echo "running $elf" &&
            "$elf"
        ) >"/work/$log" 2>&1
        if [ $? -eq 0 ]; then
            ok=1
            break
        fi
        [ $attempt -eq 1 ] && echo "SUITE RETRY $suite"
    done
    if [ $ok -eq 1 ]; then
        echo "SUITE PASS $suite"
    else
        echo "SUITE FAIL $suite (log: $log)"
        tail -n 30 "/work/$log" | sed 's/^/    | /'
        fail=$((fail + 1))
    fi
done
echo "SUITES FAILED: $fail"
exit $fail
"""
    cmd = [
        "docker", "run", "--rm",
        "-v", f"{REPO}:/work",
        "-w", "/work",
        IMAGE,
        "bash", "-c", inner,
    ]
    proc = subprocess.run(cmd)

    print()
    print("HOST TESTS PASS" if proc.returncode == 0
          else f"HOST TESTS FAIL ({proc.returncode} suite(s))")
    return proc.returncode


if __name__ == "__main__":
    sys.exit(main())
