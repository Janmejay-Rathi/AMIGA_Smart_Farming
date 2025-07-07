#!/usr/bin/env python
import os
import random
import math

# Parameters
num_weeds = 40
spacing = 0.56  # spacing between weeds (meters)
angle_deg = 85
angle_rad = math.radians(angle_deg)

dx = spacing * math.cos(angle_rad)
dy = spacing * math.sin(angle_rad)

# Build SDF
sdf = '<?xml version="1.0" ?>\n<sdf version="1.6">\n  <model name="farm_weeds">\n    <static>true</static>\n'

for i in range(num_weeds):
    x = round(i * dx, 3)
    y = round(i * dy, 3)
    name = f"weed_{i}"
    scale = round(random.uniform(0.2, 0.7), 3)

    sdf += f'''
    <link name="weed_{name}">
      <pose>{x} {y} 0 0 0 0</pose>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://weed/meshes/weed.dae</uri>
            <scale>{scale} {scale} {scale}</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>0.1 0.5 0.1 1</ambient>
          <diffuse>0.1 0.6 0.1 1</diffuse>
        </material>
      </visual>
    </link>
    '''

sdf += '  </model>\n</sdf>\n'

# Save the SDF
script_dir = os.path.dirname(os.path.abspath(__file__))
pkg_root = os.path.abspath(os.path.join(script_dir, ".."))  # amiga_sim/
model_dir = os.path.join(pkg_root, "models", "farm_weeds")
os.makedirs(model_dir, exist_ok=True)

sdf_path = os.path.join(model_dir, "model.sdf")
with open(sdf_path, "w") as f:
    f.write(sdf)

print(f"✅ Generated 30° weed column at: {sdf_path}")
