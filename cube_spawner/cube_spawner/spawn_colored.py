#!/usr/bin/env python3
import os
import random
import time
import subprocess

# --- CONFIGURATION ---
MIN_X, MAX_X = 0.35, 0.65  # Center of table
MIN_Y, MAX_Y = -0.30, 0.30
SPAWN_Z = 0.2              # Drop height (Table is 1.0m tall, so we drop from 1.1m)

# --- SDF TEMPLATE ---
CUBE_SDF = """
<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <pose>0 0 0 0 0 0</pose> 
    <link name='link'>
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>0.0001</ixx> <ixy>0</ixy> <ixz>0</ixz>
          <iyy>0.0001</iyy> <iyz>0</iyz>
          <izz>0.0001</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry>
          <box>
            <size>0.04 0.04 0.04</size>
          </box>
        </geometry>
        <surface>
            <friction>
              <ode>
                <mu>100.0</mu>
                <mu2>100.0</mu2>
              </ode>
            </friction>
        </surface>
      </collision>
      <visual name='visual'>
        <geometry>
          <box>
            <size>0.04 0.04 0.04</size>
          </box>
        </geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

def spawn_cube(name_prefix, r, g, b, count):
    """
    Spawns 'count' number of cubes with the specific RGB color.
    """
    for i in range(1, count + 1):
        # 1. Generate Random Position
        x = random.uniform(MIN_X, MAX_X)
        y = random.uniform(MIN_Y, MAX_Y)
        
        # 2. Create Unique Name and SDF
        unique_name = f"{name_prefix}_cube_{i}"
        
        # Format the SDF with the specific color numbers
        sdf_content = CUBE_SDF.format(name=unique_name, r=r, g=g, b=b)
        
        print(f"[Spawner] Spawning {unique_name} (Color: {r} {g} {b}) at X={x:.2f}, Y={y:.2f}")

        # 3. Call the ROS 2 / Ignition Create service
        subprocess.run([
            "ros2", "run", "ros_gz_sim", "create",
            "-string", sdf_content,
            "-name", unique_name,
            "-allow_renaming", "true",
            "-x", str(x),
            "-y", str(y),
            "-z", str(SPAWN_Z)
        ])
        
        # Small delay to prevent physics overlap issues
        time.sleep(0.2)

def main():
    print("--- Multi-Color Spawner Started ---")
    print("Spawning 3 Red, 3 Blue, 3 Green cubes...")

    # 1. Spawn RED Cubes (Dark Red)
    spawn_cube("red", 0.8, 0.0, 0.0, 3)

    # 2. Spawn GREEN Cubes (Dark Green)
    spawn_cube("green", 0.0, 0.8, 0.0, 3)

    # 3. Spawn BLUE Cubes (Dark Blue)
    spawn_cube("blue", 0.0, 0.0, 0.8, 3)

    print("--- Batch Spawning Complete ---")
    print("--- start recognizing now ---")

if __name__ == "__main__":
    main()