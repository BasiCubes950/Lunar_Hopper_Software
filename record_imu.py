
import serial
import csv
import time
from datetime import datetime
from pathlib import Path
from serial.tools import list_ports

HEADER = [
    "time_ms",
    "orientation_x",
    "orientation_y",
    "orientation_z",
    "accel_x",
    "accel_y",
    "accel_z",
]
START_TIMEOUT_SECONDS = 30
RECORDING_TIMEOUT_SECONDS = 30


def parse_sample(line):
    values = next(csv.reader([line]))
    if len(values) != len(HEADER):
        return None

    try:
        int(values[0])
        for value in values[1:]:
            float(value)
    except ValueError:
        return None

    return values


# Find Arduino
ports = list(list_ports.comports())

print("Available ports:")
for i, port in enumerate(ports):
    print(f"{i}: {port.device}")

if not ports:
    raise RuntimeError("No Arduino ports found!")

try:
    port_index = int(input("Select port: "))
except ValueError as error:
    raise ValueError("Port selection must be a number.") from error

if not 0 <= port_index < len(ports):
    raise ValueError(f"Port selection must be between 0 and {len(ports) - 1}.")

PORT = ports[port_index].device

# Connect
arduino = serial.Serial(PORT, 9600, timeout=1)
rows = []
recording = False
recording_ended = False
start_deadline = None
recording_deadline = None
malformed_lines = 0

try:
    time.sleep(2)
    arduino.reset_input_buffer()

    print("Connected to:", PORT)
    input("Press Enter to start recording...")

    print("Sending command 'a'...")
    arduino.write(b'a')
    arduino.flush()
    start_deadline = time.monotonic() + START_TIMEOUT_SECONDS

    while True:
        now = time.monotonic()
        if not recording and now >= start_deadline:
            print("ERROR: Timed out waiting for Arduino to start!")
            break
        if recording and now >= recording_deadline:
            print("ERROR: Timed out waiting for Arduino to finish!")
            break

        raw = arduino.readline()

        if not raw:
            continue

        line = raw.decode("utf-8", errors="replace").strip()

        print("Received:", repr(line))

        if line == "START":
            recording = True
            recording_deadline = time.monotonic() + RECORDING_TIMEOUT_SECONDS
            print("Recording started!")
            continue

        if line == "END":
            recording_ended = True
            print("Recording ended!")
            break

        if recording and line != ",".join(HEADER):
            values = parse_sample(line)
            if values is None:
                malformed_lines += 1
            else:
                rows.append(values)

finally:
    arduino.close()

# Save CSV if data was received
if rows:
    output_dir = Path.home() / "Desktop"
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    filename = output_dir / f"data_{timestamp}.csv"

    with filename.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(HEADER)
        writer.writerows(rows)

    if recording_ended:
        print("\nRecording complete!")
    else:
        print("\nRecording incomplete; saved the samples collected so far.")
    print("Samples:", len(rows))
    print("Saved to:", filename)
    if malformed_lines:
        print("Ignored malformed lines:", malformed_lines)

else:
    print("No valid data collected. CSV not created.")
