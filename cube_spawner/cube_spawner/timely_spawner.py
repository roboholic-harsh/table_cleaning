import os
import random
import time
import subprocess

# --- CONFIGURATION ---
BLOCK_NAME = "red_block"
SPAWN_INTERVAL = 100.0
MIN_X, MAX_X = 0.35, 0.70 
MIN_Y, MAX_Y = -0.35, 0.35
SPAWN_Z = 0.5            # Drop height (above table)

# SDF Definition of a Red Cube
# Note: We removed the {x} {y} {z} from here because we will pass them in the command instead
cube_sdf = """
<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <pose>0 0 0 0 0 0</pose> <link name='link'>
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
                <mu>1.0</mu>
                <mu2>1.0</mu2>
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
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          </material>
      </visual>
    </link>
  </model>
</sdf>
"""

def spawn_block():
    # 1. Generate Random Position
    x = random.uniform(MIN_X, MAX_X)
    y = random.uniform(MIN_Y, MAX_Y)
  
  
    #x,y = 0.35,0.35
    #x,y = 0.35,-0.35
    #x,y = 0.85,0.35
    #x,y = 0.85,-0.35
    
    # 2. Format the SDF with unique name
    unique_name = f"{BLOCK_NAME}_{int(time.time())}"
    sdf_content = cube_sdf.format(name=unique_name)
    
    print(f"[Spawner] Spawning {unique_name} at X={x:.2f}, Y={y:.2f}")


    # 3. Call the ROS 2 / Ignition Create service
    # FIX: We now explicitly pass -x, -y, -z to force the position
    subprocess.run([
        "ros2", "run", "ros_gz_sim", "create",
        "-string", sdf_content,
        "-name", unique_name,
        "-allow_renaming", "true",
        "-x", str(x),
        "-y", str(y),
        "-z", str(SPAWN_Z)
    ])


def main():
    print("--- Block Spawner Started ---")
    print(f"Spawning a block every {SPAWN_INTERVAL} seconds...")
    
    try:
        while True:
            spawn_block()
            time.sleep(SPAWN_INTERVAL)
    except KeyboardInterrupt:
        print("\nSpawner stopped.")

if __name__ == "__main__":
    main()