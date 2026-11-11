#!/usr/bin/env python3
"""
imu_log_to_github.py - record IMU readings from the Arduino into CSV logs, then (optionally)
commit and push them to your GitHub repo.

Pairs with imu_readings.ino, which prints:
    time_s,accel_x_mps2,accel_y_mps2,accel_z_mps2,gyro_x_dps,gyro_y_dps,gyro_z_dps
Each saved row also gets pc_time (the Mac's clock), so runs can be matched to notes/videos.

Setup (once)
    pip3 install pyserial
    Put this script inside your cloned GitHub repo (e.g. repo/tools/) if you want --push.

Examples
    python3 imu_log_to_github.py                         record 10 s -> ~/Downloads/imu_readings/2026-11-10_195500_run.csv
    python3 imu_log_to_github.py --seconds 0             record until Ctrl+C
    python3 imu_log_to_github.py --name still_bias --seconds 60
    python3 imu_log_to_github.py --name tilt30 --seconds 30 --push    record, then git add/commit/push
    python3 imu_log_to_github.py --push-only             push logs already recorded but not pushed yet
    python3 imu_log_to_github.py --list                  show serial ports

Big data
    * A new file is started automatically every --max-mb (default 45 MB) because GitHub rejects
      files over 100 MB and warns over 50 MB. At 50 readings/s a file grows about 12 MB per hour.
    * --gzip saves .csv.gz instead (about 3-4x smaller; pandas reads it directly).
    * If the logs folder grows past ~1 GB, switch the repo to Git LFS for *.csv / *.csv.gz.

Close the Arduino IDE's Serial Monitor / Plotter first: only one program can use the port.
"""
import argparse
import csv
import gzip
import os
import subprocess
import sys
import time
from datetime import datetime

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial is not installed. Run:  pip3 install pyserial")

# Where logs are saved, whatever folder Terminal is in. Change this one line to move them.
LOG_FOLDER = os.path.expanduser("~/Downloads/imu_readings")

DEFAULT_HEADER = ["time_s", "accel_x_mps2", "accel_y_mps2", "accel_z_mps2",
                  "gyro_x_dps", "gyro_y_dps", "gyro_z_dps"]


# ----------------------------------------------------------------------------- serial
def find_arduino_port():
    hints = ("usbserial", "usbmodem", "wchusbserial", "arduino", "ch340", "ch341", "ftdi", "ttyusb", "ttyacm")
    for p in serial.tools.list_ports.comports():
        text = f"{p.device} {p.description} {p.manufacturer or ''}".lower()
        if any(h in text for h in hints) and "bluetooth" not in text:
            return p.device
    return None


def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


# ----------------------------------------------------------------------------- output files
class RotatingCsv:
    """CSV writer that starts a new part file when the current one reaches max_mb."""

    def __init__(self, folder, base, header, max_mb, use_gzip):
        self.folder, self.base, self.header = folder, base, header
        self.max_bytes = int(max_mb * 1024 * 1024)
        self.use_gzip = use_gzip
        self.part, self.files, self.f, self.writer = 0, [], None, None
        self._open_next()

    def _open_next(self):
        if self.f:
            self.f.close()
        self.part += 1
        suffix = "" if self.part == 1 else f"_part{self.part}"
        ext = ".csv.gz" if self.use_gzip else ".csv"
        path = os.path.join(self.folder, f"{self.base}{suffix}{ext}")
        self.f = gzip.open(path, "wt", newline="") if self.use_gzip else open(path, "w", newline="")
        self.writer = csv.writer(self.f)
        self.writer.writerow(self.header)
        self.files.append(path)
        self.path = path
        if self.part > 1:
            print(f"\n  file limit reached -> continuing in {path}")

    def write(self, row):
        self.writer.writerow(row)
        self.f.flush()                                   # nothing lost on Ctrl+C or unplug
        if os.path.getsize(self.path) >= self.max_bytes:
            self._open_next()

    def close(self):
        if self.f:
            self.f.close()


# ----------------------------------------------------------------------------- git
def git(args, cwd):
    return subprocess.run(["git"] + args, cwd=cwd, capture_output=True, text=True)


def push_to_github(paths, folder, message):
    """git add + commit + push the given files. Returns True on success."""
    repo = git(["rev-parse", "--show-toplevel"], folder)
    if repo.returncode != 0:
        print("Not inside a git repository, so nothing was pushed.\n"
              "Clone your GitHub repo and run this script (or set --folder) inside it.")
        return False
    root = repo.stdout.strip()

    for p in paths:
        size_mb = os.path.getsize(p) / 1024 / 1024
        if size_mb > 95:
            print(f"{p} is {size_mb:.0f} MB: GitHub rejects files over 100 MB. Use --gzip or a smaller --max-mb.")
            return False

    add = git(["add"] + [os.path.abspath(p) for p in paths], root)
    if add.returncode != 0:
        print("git add failed:\n" + add.stderr)
        return False
    commit = git(["commit", "-m", message], root)
    if commit.returncode != 0 and "nothing to commit" not in (commit.stdout + commit.stderr):
        print("git commit failed:\n" + commit.stderr + commit.stdout)
        return False
    print("Pushing to GitHub...")
    push = git(["push"], root)
    if push.returncode != 0:
        print("git push failed (saved and committed locally; run 'git pull --rebase' then 'git push'):\n"
              + push.stderr)
        return False
    print("Pushed: " + ", ".join(os.path.relpath(p, root) for p in paths))
    return True


def unpushed_logs(folder):
    """Log files in folder that git sees as new or modified."""
    status = git(["status", "--porcelain", "--untracked-files=all", "--", "."], folder)
    if status.returncode != 0:
        return []
    files = []
    for line in status.stdout.splitlines():
        path = line[3:].strip()
        if path.endswith((".csv", ".csv.gz")):
            root = git(["rev-parse", "--show-toplevel"], folder).stdout.strip()
            files.append(os.path.join(root, path))
    return files


# ----------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description="Record Arduino IMU readings to CSV and push to GitHub.")
    ap.add_argument("--port", help="serial port (found automatically if left out)")
    ap.add_argument("--baud", type=int, default=115200, help="must match Serial.begin() (default 115200)")
    ap.add_argument("--name", default="run", help="label for this run, used in the file name")
    ap.add_argument("--folder", default=LOG_FOLDER, help=f"where logs are saved (default {LOG_FOLDER})")
    ap.add_argument("--seconds", type=float, default=10, help="record this many seconds (default 10; 0 = until Ctrl+C)")
    ap.add_argument("--max-mb", type=float, default=45, help="start a new file after this size (default 45)")
    ap.add_argument("--gzip", action="store_true", help="save compressed .csv.gz files")
    ap.add_argument("--push", action="store_true", help="git add, commit and push the new file(s) when done")
    ap.add_argument("--push-only", action="store_true", help="don't record; push logs not yet on GitHub")
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    a = ap.parse_args()

    if a.list:
        for p in serial.tools.list_ports.comports():
            print(f"{p.device:35s} {p.description}")
        return

    os.makedirs(a.folder, exist_ok=True)

    if a.push_only:
        files = unpushed_logs(a.folder)
        if not files:
            print("No new log files to push.")
            return
        push_to_github(files, a.folder, f"IMU logs: {len(files)} file(s)")
        return

    port = a.port or find_arduino_port()
    if not port:
        sys.exit("No Arduino found. Plug it in, close the Serial Monitor, or pass --port (see --list).")
    try:
        ser = serial.Serial(port, a.baud, timeout=0.5)
    except serial.SerialException as e:
        sys.exit(f"Could not open {port}: {e}\nIs the Arduino IDE's Serial Monitor still open?")

    safe_name = "".join(c if c.isalnum() or c in "-_" else "_" for c in a.name)
    base = datetime.now().strftime("%Y-%m-%d_%H%M%S") + f"_{safe_name}"

    print(f"Connected to {port} at {a.baud} baud.")
    print("The Arduino restarts now; keep the IMU still for 2-3 seconds.  Ctrl+C to stop.\n")

    header = None
    out = None
    rows, bad, start = 0, 0, None
    t_first = t_last = None

    try:
        while True:
            if a.seconds and start and time.time() - start >= a.seconds:
                break
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            if line.startswith("#"):
                print("  " + line)                         # messages from the sketch
                continue
            parts = [p.strip() for p in line.split(",")]
            if parts and parts[0] == "time_s":             # the sketch's header row
                header = parts
                continue
            if not all(is_number(p) for p in parts):
                print("  " + line)
                continue
            if out is None:                                # first reading: open the file
                header = header or DEFAULT_HEADER
                if len(parts) != len(header):
                    header = [f"v{i + 1}" for i in range(len(parts))]
                out = RotatingCsv(a.folder, base, ["pc_time"] + header, a.max_mb, a.gzip)
                start = time.time()
                print(f"Saving to {os.path.abspath(out.path)}\nColumns: {', '.join(['pc_time'] + header)}\n")
            if len(parts) != len(header):
                bad += 1                                   # a line cut off mid-transfer: skip it
                continue
            out.write([datetime.now().isoformat(timespec="milliseconds")] + parts)
            rows += 1
            if header[0] == "time_s":
                t_last = float(parts[0])
                t_first = t_last if t_first is None else t_first
            if rows % 25 == 0:
                vals = dict(zip(header, parts))
                print(f"\r{rows:7d} rows | t={vals.get('time_s', '?')} s  "
                      f"a=({vals.get('accel_x_mps2', '?')}, {vals.get('accel_y_mps2', '?')}, "
                      f"{vals.get('accel_z_mps2', '?')}) m/s^2  "
                      f"g=({vals.get('gyro_x_dps', '?')}, {vals.get('gyro_y_dps', '?')}, "
                      f"{vals.get('gyro_z_dps', '?')}) deg/s   ", end="", flush=True)
    except KeyboardInterrupt:
        pass
    except serial.SerialException as e:
        print(f"\nConnection lost: {e}")
    finally:
        ser.close()
        if out:
            out.close()

    if not out:
        sys.exit("\nNo readings received. Check the baud rate and that the sketch is printing.")

    print(f"\n\nSaved {rows} rows to: " + ", ".join(os.path.abspath(p) for p in out.files)
          + (f"  ({bad} cut-off lines skipped)" if bad else ""))
    if t_first is not None and t_last > t_first:
        span = t_last - t_first
        print(f"Arduino time covered: {span:.2f} s  ->  {(rows - 1) / span:.1f} readings per second "
              f"(target 100; much lower means lines were lost)")

    if sys.platform == "darwin" and not a.push:
        subprocess.run(["open", "-R", os.path.abspath(out.files[-1])])   # show the file in Finder
    if a.push:
        push_to_github(out.files, a.folder, f"IMU log {base} ({rows} rows)")
    else:
        print("Not pushed. Add --push next time, or run:  python3 imu_log_to_github.py --push-only")


if __name__ == "__main__":
    main()
