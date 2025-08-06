#!/usr/bin/env python
import os
import random

# Define weed layout
rows, cols = 20, 10
row_spacing = 1.18    # space between rows (Y-direction) 22 inches
col_spacing = 1.87    # space between columns (X-direction) 35 inches
quadrants = ["top_left", "top_right", "bottom_left", "bottom_right"]  # Add others as needed

# Get signs for each quadrant
def get_signs_for_quadrant(q):
    return {
        "top_left": (-1, 1),
        "top_right": (1, 1),
        "bottom_left": (-1, -1),
        "bottom_right": (1, -1)
    }[q]

# Build SDF
sdf = '<?xml version="1.0" ?>\n<sdf version="1.6">\n  <model name="farm_weeds">\n    <static>true</static>\n'
for q in quadrants:
    x_sign, y_sign = get_signs_for_quadrant(q)
    for i in range(rows):
        for j in range(cols):
            x = (j + 1) * col_spacing * x_sign
            y = (i + 1) * row_spacing * y_sign
            name = f"{q}_{i}_{j}"
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

# Get absolute path to package directory
script_dir = os.path.dirname(os.path.abspath(__file__))
pkg_root = os.path.abspath(os.path.join(script_dir, ".."))  # amiga_sim/
model_dir = os.path.join(pkg_root, "models", "farm_weeds")
os.makedirs(model_dir, exist_ok=True)

sdf_path = os.path.join(model_dir, "model.sdf")
with open(sdf_path, "w") as f:
    f.write(sdf)

print(f"✅ Generated: {sdf_path}")
