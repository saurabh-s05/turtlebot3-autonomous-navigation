import json
import subprocess
import time

obstacles = [
    ("moving_obs_1", 1.45, 1.05),
    ("moving_obs_2", -0.5, 0),
    ("moving_obs_3", -1.45, -1.5),
    ("moving_obs_4", 0.55, 0.5),
]

for name, x, y in obstacles:

    sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <pose>0 0 0 0 0 0</pose>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box><size>0.15 0.15 0.35</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>0.10 0.10 0.35</size></box>
        </geometry>
        <material>
          <ambient>0.9 0.05 0.05 1</ambient>
          <diffuse>0.9 0.05 0.05 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

    request = (
        f"sdf: {json.dumps(sdf)} "
        f"name: \"{name}\" "
        f"pose: {{position: {{x: {x}, y: {y}, z: 0.175}} "
        f"orientation: {{w: 1.0}}}}"
    )

    print(f"Spawning {name} at ({x}, {y})")

    result = subprocess.run(
        [
            "gz", "service",
            "-s", "/world/default/create",
            "--reqtype", "gz.msgs.EntityFactory",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000",
            "--req", request,
        ],
        capture_output=True,
        text=True,
    )

    print(result.stdout)
    if result.returncode != 0:
        print("ERROR:", result.stderr)
        break

    time.sleep(1)

print("Spawning process completed.")

