import math
import subprocess
import time
import signal

SERVICE = "/world/default/set_pose"

OBSTACLES = [
    {"name": "moving_obs_1", "x": 1.45, "y": 1.05, "axis": "x"},
    {"name": "moving_obs_2", "x": -0.5, "y": 0.0, "axis": "y"},
    {"name": "moving_obs_3", "x": -1.45, "y": -1.5, "axis": "x"},
    {"name": "moving_obs_4", "x": 0.55, "y": 0.5, "axis": "y"},
]

SPEED = 0.08
AMPLITUDE = 0.30
UPDATE_INTERVAL = 0.5
Z = 0.175
MAX_RETRIES = 3

running = True


def stop_handler(sig, frame):
    global running
    running = False


signal.signal(signal.SIGINT, stop_handler)
signal.signal(signal.SIGTERM, stop_handler)


def set_pose(name, x, y):

    request = (
        f'name: "{name}" '
        f'position: {{x: {x:.4f}, y: {y:.4f}, z: {Z}}} '
        'orientation: {w: 1.0}'
    )

    for attempt in range(MAX_RETRIES):

        try:
            result = subprocess.run(
                [
                    "gz", "service",
                    "-s", SERVICE,
                    "--reqtype", "gz.msgs.Pose",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "3000",
                    "--req", request
                ],
                capture_output=True,
                text=True,
                timeout=5
            )

            if result.returncode == 0 and "data: true" in result.stdout:
                return True

            print(f"{name}: retry {attempt + 1}")

        except subprocess.TimeoutExpired:
            print(f"{name}: service timeout, retry {attempt + 1}")

        time.sleep(0.2)

    print(f"WARNING: Could not update {name}; continuing.")
    return False


def triangle_wave(t, phase):

    q = (SPEED * t / (4.0 * AMPLITUDE) + phase) % 1.0

    return AMPLITUDE * (1.0 - 4.0 * abs(q - 0.5))


print("Four moving obstacles started.")
print("All obstacles move back and forth.")
print("Press Ctrl+C to stop.")

start = time.monotonic()

try:

    while running:

        t = time.monotonic() - start

        for i, obs in enumerate(OBSTACLES):

            if not running:
                break

            offset = triangle_wave(t, i * 0.25)

            x = obs["x"]
            y = obs["y"]

            if obs["axis"] == "x":
                x += offset
            else:
                y += offset

            success = set_pose(obs["name"], x, y)

            if success:
                print(
                    f'{obs["name"]}: '
                    f'x={x:.2f}, y={y:.2f}'
                )

        time.sleep(UPDATE_INTERVAL)

except KeyboardInterrupt:
    pass

finally:
    print("Movement experiment stopped.")
