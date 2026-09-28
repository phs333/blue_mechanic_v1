"""Compila e roda os testes C de host (planejador de movimento) sem ESP-IDF.

Usa `python -m ziglang cc` (pip install ziglang) ou o gcc/clang do PATH.
Também é executado pela suíte unittest (tests/test_host_c.py).
"""

import os
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SOURCES = [
    os.path.join(ROOT, "tests", "host", "test_motion_profile.c"),
    os.path.join(ROOT, "main", "motion_profile.c"),
]


def find_compiler():
    for cc in ("gcc", "clang", "cc"):
        if shutil.which(cc):
            return [cc]
    try:
        import ziglang  # noqa: F401
        return [sys.executable, "-m", "ziglang", "cc"]
    except ImportError:
        return None


def build_and_run():
    cc = find_compiler()
    if cc is None:
        return None, "compilador C não encontrado (instale gcc ou `pip install ziglang`)"
    out_dir = tempfile.mkdtemp(prefix="bm_host_")
    exe = os.path.join(out_dir, "test_motion_profile.exe")
    cmd = cc + ["-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-I", os.path.join(ROOT, "main"),
                "-o", exe] + SOURCES + ["-lm"]
    build = subprocess.run(cmd, capture_output=True, text=True)
    if build.returncode != 0:
        return False, build.stdout + build.stderr
    # O antivírus do Windows pode travar por instantes um .exe recém-criado (PermissionError)
    for attempt in range(5):
        try:
            run = subprocess.run([exe], capture_output=True, text=True)
            return run.returncode == 0, run.stdout + run.stderr
        except OSError as exc:
            if attempt == 4:
                return False, f"não foi possível executar {exe}: {exc}"
            time.sleep(0.5 * (attempt + 1))


if __name__ == "__main__":
    ok, output = build_and_run()
    print(output)
    sys.exit(0 if ok else 1)
