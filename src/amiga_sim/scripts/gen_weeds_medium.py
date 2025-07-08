#!/usr/bin/env python
import os
import random
import math

# Parameters
num_weeds = 80
radius = 25.0  # meters
total_arc_degrees = 130
total_arc_radians = math.radians(total_arc_degrees)
angle_step_rad = total_arc_radians / max(num_weeds - 1, 1)

# Compute offset from the first point so it starts at origin
start_theta = 0
start_x = radius * math.cos(start_theta)
start_y = radius * math.sin(start_theta)

# Build SDF
sdf = '<?xml version="1.0" ?>\n<sdf version="1.6">\n  <model name="farm_weeds">\n    <static>true</static>\n'

for i in range(num_weeds):
    theta = angle_step_rad * i
    x = radius * math.cos(theta) - start_x
    y = radius * math.sin(theta) - start_y
    x = round(x, 3)
    y = round(y, 3)

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

print(f"✅ Generated curved weed arc starting at origin at: {sdf_path}")
