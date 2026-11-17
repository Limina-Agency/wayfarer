"""Runs one command and records its wall time, CPU time and peak memory.

    python measure.py <out.json> <label> [--stdout <file>] [--log <file>] -- <command> [args...]

Writes one JSON object to out.json (label, command, exit code, wall_s,
user_s, sys_s, max_rss_mib) and prints a one-line summary on stderr. Peak
memory is the largest resident set of any process the command waited for
(getrusage of the children), so a command that forks workers reports its
largest worker, not their sum. The command's stdout goes to --stdout, its
stderr to --log (and its stdout too, without --stdout); both default to
passing through.
"""

import json
import resource
import subprocess
import sys
import time


def main():
    args = sys.argv[1:]
    if "--" not in args or args.index("--") < 2:
        sys.exit("usage: measure.py <out.json> <label> [--stdout <file>] [--log <file>] -- <command> [args...]")
    split = args.index("--")
    (out_path, label, *options), command = args[:split], args[split + 1 :]
    files = dict(zip(options[::2], options[1::2]))
    log = open(files["--log"], "w") if "--log" in files else None
    stdout = open(files["--stdout"], "w") if "--stdout" in files else log
    started = time.perf_counter()
    status = subprocess.run(command, stdout=stdout, stderr=log).returncode
    wall = time.perf_counter() - started
    usage = resource.getrusage(resource.RUSAGE_CHILDREN)
    # ru_maxrss is KiB on Linux, bytes on macOS.
    rss_kib = usage.ru_maxrss / (1024 if sys.platform == "darwin" else 1)
    record = {
        "label": label,
        "command": command,
        "exit": status,
        "wall_s": round(wall, 3),
        "user_s": round(usage.ru_utime, 3),
        "sys_s": round(usage.ru_stime, 3),
        "max_rss_mib": round(rss_kib / 1024, 1),
    }
    with open(out_path, "w") as out:
        json.dump(record, out, indent=1)
    print(
        f"{label}: {wall:.1f} s wall, {usage.ru_utime + usage.ru_stime:.1f} s CPU,"
        f" peak {record['max_rss_mib']:.0f} MiB" + ("" if status == 0 else f", exit {status}"),
        file=sys.stderr,
    )
    sys.exit(status)


if __name__ == "__main__":
    main()
