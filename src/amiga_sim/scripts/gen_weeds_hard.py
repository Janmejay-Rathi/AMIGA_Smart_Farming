#!/usr/bin/env python
import os
import random
import math

# Parameters
num_weeds = 40
spacing = 0.56       # spacing between weeds (meters)
amplitude = 0.8      # sine wave height
frequency = 0.4      # lower frequency for smoother curves

# Build SDF
sdf = '<?xml version="1.0" ?>\n<sdf version="1.6">\n  <model name="farm_weeds">\n    <static>true</static>\n'

for i in range(num_weeds):
    raw_x = i * spacing
    raw_y = amplitude * math.sin(frequency * raw_x)

    x = round(-raw_y, 3)  # 90° counter-clockwise
    y = round(raw_x, 3)

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

print(f"✅ Generated sine wave weed path (starting at origin) at: {sdf_path}")
