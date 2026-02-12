#!/usr/bin/env python3

# --- 1. NO EVENTLET IMPORTS HERE --- 
import threading # Standard threading
import time
import os
import csv
import cv2
import copy
import random
import subprocess
import math
import base64
import sys

# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIK
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener

# Web Server Imports
from flask import Flask, render_template_string, request
from flask_socketio import SocketIO, emit

# --- CONFIGURATION ---
DATASET_ROOT = "lerobot_raw_dataset"
RECORD_RATE_HZ = 30
WEB_STREAM_FPS = 15  
CARTESIAN_STEP = 0.01
JOINT_STEP = 0.05  # Radians per tick

# --- SAFETY LIMITS ---
JOINT_LIMITS = [
    [-2.89, 2.89], [-1.76, 1.76], [-2.89, 2.89], [-3.07, -0.06], 
    [-2.89, 2.89], [-0.01, 3.75], [-2.89, 2.89]
]
CARTESIAN_LIMITS = {
    'x': (0.2, 0.8), 'y': (-0.6, 0.6), 'z': (0.1, 0.8)
}

# --- TOPICS ---
TOPIC_CAM_OVERHEAD = "/camera/overhead_camera/image_raw"     
TOPIC_CAM_HAND     = "/camera/hand_camera/image_raw" 
TOPIC_JOINT_STATES = "/joint_states"

# --- HTML TEMPLATE ---
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>LeRobot Web Teleop</title>
    <style>
        body { font-family: sans-serif; background: #222; color: white; text-align: center; margin: 0; padding: 10px; user-select: none; }
        .container { max-width: 600px; margin: auto; padding-bottom: 50px; }
        .cam-container { display: flex; justify-content: center; gap: 5px; margin-bottom: 10px; }
        img { width: 48%; border: 2px solid #555; border-radius: 5px; background: #000; min-height: 150px; }
        .panel { background: #333; padding: 10px; border-radius: 8px; margin-bottom: 10px; }
        h3 { margin: 5px 0 10px 0; border-bottom: 1px solid #555; padding-bottom: 5px; font-size: 16px; }
        button { 
            width: 100%; padding: 15px; margin: 3px 0; 
            font-size: 16px; font-weight: bold; border: none; border-radius: 8px; 
            cursor: pointer; touch-action: manipulation; -webkit-tap-highlight-color: transparent;
        }
        .btn-green { background: #28a745; color: white; }
        .btn-red { background: #dc3545; color: white; }
        .btn-blue { background: #007bff; color: white; }
        .btn-orange { background: #fd7e14; color: white; }
        .btn-purple { background: #6f42c1; color: white; }
        .btn-gray { background: #6c757d; color: white; }
        button:active { opacity: 0.6; transform: scale(0.98); }
        .grid-3 { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 8px; align-items: center; }
        .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; align-items: center; }
        input[type=range] { width: 100%; margin: 15px 0; }
        #status { font-weight: bold; color: #00ff00; margin-bottom: 5px; font-size: 14px; }
        .console-log { font-size: 10px; color: #777; margin-top: 5px; }
    </style>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
</head>
<body>
    <div class="container">
        <div id="status">Connecting...</div>
        <div class="cam-container">
            <img id="img_overhead" src="" alt="Overhead">
            <img id="img_hand" src="" alt="Hand">
        </div>
        
        <div class="panel">
            <h3>Data Collection</h3>
            <div class="grid-3">
                <button class="btn-green" onclick="toggleRec()" id="btn_rec">REC START</button>
                <div></div>
                <button class="btn-red" onclick="discard()">DISCARD</button>
            </div>
            <div id="rec_info" class="console-log">Ready</div>
        </div>

        <div class="panel">
            <h3>Joint Control</h3>
            <div class="grid-2">
                <button class="btn-gray" onpointerdown="startJoint(0, -1)" onpointerup="stopMove()" onpointerleave="stopMove()">J1 -</button>
                <button class="btn-gray" onpointerdown="startJoint(0, 1)" onpointerup="stopMove()" onpointerleave="stopMove()">J1 +</button>
                <button class="btn-gray" onpointerdown="startJoint(1, -1)" onpointerup="stopMove()" onpointerleave="stopMove()">J2 -</button>
                <button class="btn-gray" onpointerdown="startJoint(1, 1)" onpointerup="stopMove()" onpointerleave="stopMove()">J2 +</button>
                <button class="btn-gray" onpointerdown="startJoint(2, -1)" onpointerup="stopMove()" onpointerleave="stopMove()">J3 -</button>
                <button class="btn-gray" onpointerdown="startJoint(2, 1)" onpointerup="stopMove()" onpointerleave="stopMove()">J3 +</button>
                <button class="btn-gray" onpointerdown="startJoint(3, -1)" onpointerup="stopMove()" onpointerleave="stopMove()">J4 -</button>
                <button class="btn-gray" onpointerdown="startJoint(3, 1)" onpointerup="stopMove()" onpointerleave="stopMove()">J4 +</button>
                <button class="btn-gray" onpointerdown="startJoint(4, -1)" onpointerup="stopMove()" onpointerleave="stopMove()">J5 -</button>
                <button class="btn-gray" onpointerdown="startJoint(4, 1)" onpointerup="stopMove()" onpointerleave="stopMove()">J5 +</button>
                <button class="btn-gray" onpointerdown="startJoint(5, -1)" onpointerup="stopMove()" onpointerleave="stopMove()">J6 -</button>
                <button class="btn-gray" onpointerdown="startJoint(5, 1)" onpointerup="stopMove()" onpointerleave="stopMove()">J6 +</button>
                <button class="btn-gray" onpointerdown="startJoint(6, -1)" onpointerup="stopMove()" onpointerleave="stopMove()">J7 -</button>
                <button class="btn-gray" onpointerdown="startJoint(6, 1)" onpointerup="stopMove()" onpointerleave="stopMove()">J7 +</button>
            </div>
        </div>

        <div class="panel">
            <h3>Cartesian Control</h3>
            <label>Speed: <span id="speed_val" class="slider-val">0.6s</span></label>
            <input type="range" id="speed_slider" min="0.1" max="1.5" step="0.1" value="0.6" oninput="updateSpeedLabel(this.value)">
            <div class="grid-3">
                <button class="btn-gray" onpointerdown="startCart('x', -1)" onpointerup="stopMove()" onpointerleave="stopMove()">X -</button>
                <span>X</span>
                <button class="btn-gray" onpointerdown="startCart('x', 1)" onpointerup="stopMove()" onpointerleave="stopMove()">X +</button>
                <button class="btn-gray" onpointerdown="startCart('y', -1)" onpointerup="stopMove()" onpointerleave="stopMove()">Y -</button>
                <span>Y</span>
                <button class="btn-gray" onpointerdown="startCart('y', 1)" onpointerup="stopMove()" onpointerleave="stopMove()">Y +</button>
                <button class="btn-gray" onpointerdown="startCart('z', -1)" onpointerup="stopMove()" onpointerleave="stopMove()">Z -</button>
                <span>Z</span>
                <button class="btn-gray" onpointerdown="startCart('z', 1)" onpointerup="stopMove()" onpointerleave="stopMove()">Z +</button>
            </div>
            <button class="btn-blue" onclick="sendCommand('align')">ALIGN VERTICAL</button>
        </div>

        <div class="panel">
            <div class="grid-3">
                <button class="btn-orange" onclick="sendCommand('home')">HOME</button>
                <button class="btn-purple" onclick="spawn()">BLOCK</button>
                <button class="btn-orange" onclick="sendCommand('dustbin')">BIN</button>
            </div>
        </div>

        <div class="panel">
            <h3>Gripper</h3>
            <input type="range" min="0" max="0.04" step="0.001" value="0.04" oninput="sendCommand('gripper', {val: this.value})">
        </div>
    </div>

    <script>
        var socket = io();
        var moveInterval = null;
        var isRecording = false;

        socket.on('connect', () => { document.getElementById("status").innerText = "Connected!"; });
        socket.on('update_image', (msg) => {
            let img = document.getElementById(msg.type == 'overhead' ? "img_overhead" : "img_hand");
            if(img) img.src = "data:image/jpeg;base64," + msg.data;
        });
        socket.on('status_update', (msg) => {
            document.getElementById("status").innerText = msg.text;
            document.getElementById("status").style.color = msg.color;
            if(msg.rec_info) document.getElementById("rec_info").innerText = msg.rec_info;
        });

        function sendCommand(cmd, data={}) { socket.emit('control_cmd', {command: cmd, ...data}); }

        function toggleRec() {
            var btn = document.getElementById("btn_rec");
            if(!isRecording) {
                isRecording = true;
                sendCommand('rec_start');
                btn.innerText = "STOP & SAVE"; btn.className = "btn-red";
            } else {
                isRecording = false;
                let name = prompt("Episode Name:");
                if(name !== null) sendCommand('rec_stop', {name: name});
                else sendCommand('rec_discard');
                btn.innerText = "REC START"; btn.className = "btn-green";
            }
        }
        function discard() { if(isRecording) toggleRec(); sendCommand('rec_discard'); }
        function spawn() { sendCommand('spawn'); }
        
        function startCart(axis, dir) {
            if(moveInterval) return;
            let speed = document.getElementById("speed_slider").value;
            sendCommand('move_cartesian', {axis: axis, dir: dir, speed: speed});
            moveInterval = setInterval(() => { sendCommand('move_cartesian', {axis: axis, dir: dir, speed: speed}); }, parseFloat(speed) * 1000); 
        }

        function startJoint(idx, dir) {
            if(moveInterval) return;
            sendCommand('move_joint', {idx: idx, dir: dir});
            moveInterval = setInterval(() => { sendCommand('move_joint', {idx: idx, dir: dir}); }, 100); 
        }

        function stopMove() { if(moveInterval) { clearInterval(moveInterval); moveInterval = null; } }
        function updateSpeedLabel(val) { document.getElementById("speed_val").innerText = val + "s"; }
    </script>
</body>
</html>
"""

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
# FORCE THREADING MODE
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

class PandaBackend(Node):
    def __init__(self):
        super().__init__('panda_web_backend')
        
        self.arm_pub = self.create_publisher(JointTrajectory, "/panda_arm_controller/joint_trajectory", 10)
        self.hand_pub = self.create_publisher(JointTrajectory, "/hand_controller/joint_trajectory", 10)
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()
        
        self.latest_jpeg_overhead = None
        self.latest_jpeg_hand = None
        self.obs = {"overhead": None, "hand": None, "joints": None, "effort": None}
        self.recording = False
        self.episode_data = []
        self.frame_count = 0
        self.episode_idx = self.get_next_idx()

        self.create_subscription(Image, TOPIC_CAM_OVERHEAD, self.cb_cam_overhead, qos_profile_sensor_data)
        self.create_subscription(Image, TOPIC_CAM_HAND, self.cb_cam_hand, qos_profile_sensor_data)
        self.create_subscription(JointState, TOPIC_JOINT_STATES, self.cb_joints, 10)

        print("--- BACKEND READY ---")
        self.stream_thread = threading.Thread(target=self.broadcast_streams, daemon=True)
        self.stream_thread.start()

    def get_next_idx(self):
        if not os.path.exists(DATASET_ROOT): return 0
        existing = [d for d in os.listdir(DATASET_ROOT) if d.startswith("episode_")]
        if not existing: return 0
        try: return max([int(d.split("_")[-1]) for d in existing if d.split("_")[-1].isdigit()]) + 1
        except: return 0

    def cb_cam_overhead(self, msg):
        self.obs["overhead"] = msg
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            _, buf = cv2.imencode('.jpg', cv2.resize(cv_img, (320, 240)), [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            self.latest_jpeg_overhead = base64.b64encode(buf).decode('utf-8')
        except: pass

    def cb_cam_hand(self, msg):
        self.obs["hand"] = msg
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            _, buf = cv2.imencode('.jpg', cv2.resize(cv_img, (320, 240)), [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            self.latest_jpeg_hand = base64.b64encode(buf).decode('utf-8')
        except: pass

    def cb_joints(self, msg):
        target_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
        pos, eff = [], []
        try:
            for name in target_names:
                if name in msg.name:
                    idx = msg.name.index(name)
                    pos.append(msg.position[idx])
                    val = msg.effort[idx]
                    eff.append(0.0 if math.isnan(val) else val)
                else: return
            self.obs["joints"] = pos
            self.obs["effort"] = eff
        except: pass

    def broadcast_streams(self):
        rate = 1.0 / WEB_STREAM_FPS
        while rclpy.ok():
            if self.latest_jpeg_overhead:
                socketio.emit('update_image', {'type': 'overhead', 'data': self.latest_jpeg_overhead})
            if self.latest_jpeg_hand:
                socketio.emit('update_image', {'type': 'hand', 'data': self.latest_jpeg_hand})
            time.sleep(rate)

    def start_recording(self):
        self.recording = True
        self.episode_data = []
        self.frame_count = 0
        threading.Thread(target=self.record_loop, daemon=True).start()

    def record_loop(self):
        rate = 1.0/RECORD_RATE_HZ
        while self.recording and rclpy.ok():
            start = time.time()
            if self.obs["joints"] and self.obs["overhead"]:
                try:
                    cv_over = self.bridge.imgmsg_to_cv2(self.obs["overhead"], "bgr8")
                    cv_hand = self.bridge.imgmsg_to_cv2(self.obs["hand"], "bgr8") if self.obs["hand"] else None
                    snap = {
                        "timestamp": time.time(),
                        "joints": copy.deepcopy(self.obs["joints"]),
                        "effort": copy.deepcopy(self.obs["effort"]),
                        "gripper": 0.04,
                        "img_over": cv_over,
                        "img_hand": cv_hand
                    }
                    self.episode_data.append(snap)
                    self.frame_count += 1
                    if self.frame_count % 15 == 0:
                        socketio.emit('status_update', {'text': 'Recording...', 'color': 'red', 'rec_info': f"Frames: {self.frame_count}"})
                except: pass
            elapsed = time.time() - start
            if elapsed < rate: time.sleep(rate - elapsed)

    def save_recording(self, name):
        if not self.episode_data: return None
        if not name: name = f"episode_{self.episode_idx:06d}"
        path = os.path.join(DATASET_ROOT, name)
        os.makedirs(path, exist_ok=True)
        img_over_dir = os.path.join(path, "camera_overhead")
        img_hand_dir = os.path.join(path, "camera_hand")
        os.makedirs(img_over_dir, exist_ok=True)
        os.makedirs(img_hand_dir, exist_ok=True)
        
        csv_rows = []
        for i, d in enumerate(self.episode_data):
            fname = f"frame_{i:06d}.jpg"
            cv2.imwrite(os.path.join(img_over_dir, fname), d["img_over"])
            if d["img_hand"] is not None: cv2.imwrite(os.path.join(img_hand_dir, fname), d["img_hand"])
            row = {"index": i, "timestamp": d["timestamp"], "gripper_pos": d["gripper"]}
            for j, v in enumerate(d["joints"]): row[f"joint_{j+1}"] = v
            for j, v in enumerate(d["effort"]): row[f"effort_{j+1}"] = v
            csv_rows.append(row)
            
        with open(os.path.join(path, "states.csv"), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=csv_rows[0].keys())
            w.writeheader()
            w.writerows(csv_rows)
        self.episode_idx += 1
        self.episode_data = []
        return name

    # --- MOVEMENT LOGIC ---
    def solve_ik_and_move(self, pose, duration=0.5):
        if not self.ik_client.service_is_ready(): 
            print("IK Service not ready")
            return
        
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = "world"
        req.ik_request.pose_stamped.pose = pose
        req.ik_request.avoid_collisions = True
        
        if self.obs["joints"] is not None:
            req.ik_request.robot_state.joint_state.name = [f'panda_joint{i+1}' for i in range(7)]
            req.ik_request.robot_state.joint_state.position = self.obs["joints"]
        else: return

        future = self.ik_client.call_async(req)
        start = time.time()
        while not future.done():
            if time.time() - start > 1.0: return
            time.sleep(0.01)
            
        res = future.result()
        if res.error_code.val == 1:
            msg = JointTrajectory()
            msg.joint_names = [f'panda_joint{i+1}' for i in range(7)]
            p = JointTrajectoryPoint()
            p.positions = list(res.solution.joint_state.position)[:7]
            p.time_from_start = Duration(seconds=float(duration)).to_msg()
            msg.points = [p]
            self.arm_pub.publish(msg)

    def get_current_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('world', 'panda_link8', rclpy.time.Time(), rclpy.duration.Duration(seconds=0.1))
            p = Pose()
            p.position.x = t.transform.translation.x
            p.position.y = t.transform.translation.y
            p.position.z = t.transform.translation.z
            p.orientation = t.transform.rotation
            return p
        except: return None

    def publish_hand(self, val):
        msg = JointTrajectory()
        msg.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]
        p = JointTrajectoryPoint()
        p.positions = [val, val]
        p.time_from_start = Duration(seconds=0.1).to_msg()
        msg.points = [p]
        self.hand_pub.publish(msg)

node = None

@app.route('/')
def index(): return render_template_string(HTML_TEMPLATE)

@socketio.on('control_cmd')
def handle_command(data):
    global node
    cmd = data['command']
    
    if cmd == 'move_cartesian':
        axis, val, dur = data['axis'], float(data['dir']) * CARTESIAN_STEP, float(data['speed'])
        curr = node.get_current_pose()
        if curr:
            tgt = copy.deepcopy(curr)
            if axis == 'x': tgt.position.x += val
            elif axis == 'y': tgt.position.y += val
            elif axis == 'z': tgt.position.z += val
            
            # --- CARTESIAN CLAMP ---
            tgt.position.x = max(CARTESIAN_LIMITS['x'][0], min(tgt.position.x, CARTESIAN_LIMITS['x'][1]))
            tgt.position.y = max(CARTESIAN_LIMITS['y'][0], min(tgt.position.y, CARTESIAN_LIMITS['y'][1]))
            tgt.position.z = max(CARTESIAN_LIMITS['z'][0], min(tgt.position.z, CARTESIAN_LIMITS['z'][1]))
            
            node.solve_ik_and_move(tgt, dur)

    elif cmd == 'move_joint':
        idx = int(data['idx'])
        direction = float(data['dir'])
        
        current_joints = node.obs["joints"]
        if current_joints:
            target_joints = list(current_joints)
            target_joints[idx] += direction * JOINT_STEP
            
            # --- JOINT LIMIT CLAMP ---
            limit_min, limit_max = JOINT_LIMITS[idx]
            target_joints[idx] = max(limit_min, min(target_joints[idx], limit_max))
            
            msg = JointTrajectory()
            msg.joint_names = [f'panda_joint{i+1}' for i in range(7)]
            p = JointTrajectoryPoint()
            p.positions = target_joints
            p.time_from_start = Duration(seconds=0.1).to_msg()
            msg.points = [p]
            node.arm_pub.publish(msg)

    elif cmd == 'align':
        curr = node.get_current_pose()
        if curr:
            tgt = copy.deepcopy(curr)
            tgt.orientation.x = 0.9238; tgt.orientation.y = -0.3826
            tgt.orientation.z = 0.0; tgt.orientation.w = 0.0
            node.solve_ik_and_move(tgt, 1.0)

    elif cmd == 'home':
        msg = JointTrajectory()
        msg.joint_names = [f'panda_joint{i+1}' for i in range(7)]
        p = JointTrajectoryPoint()
        p.positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]
        p.time_from_start = Duration(seconds=2.0).to_msg()
        msg.points = [p]
        node.arm_pub.publish(msg)

    elif cmd == 'dustbin':
        p = Pose()
        p.position.x, p.position.y, p.position.z = 0.0, 0.5, 0.4
        p.orientation.x, p.orientation.y = 0.9238, -0.3826
        p.orientation.z, p.orientation.w = 0.0, 0.0
        node.solve_ik_and_move(p, 2.0)

    elif cmd == 'gripper':
        node.publish_hand(float(data['val']))

    elif cmd == 'spawn':
        x, y = random.uniform(0.35, 0.70), random.uniform(-0.35, 0.35)
        name = f"red_block_{int(time.time())}"
        sdf = f"""<?xml version='1.0'?><sdf version='1.6'><model name='{name}'><pose>0 0 0 0 0 0</pose><link name='link'><collision name='c'><geometry><box><size>0.04 0.04 0.04</size></box></geometry></collision><visual name='v'><geometry><box><size>0.04 0.04 0.04</size></box></geometry><material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual></link></model></sdf>"""
        subprocess.run(["ros2", "run", "ros_gz_sim", "create", "-string", sdf, "-name", name, "-allow_renaming", "true", "-x", str(x), "-y", str(y), "-z", "0.5"])

    elif cmd == 'rec_start':
        node.start_recording()
        emit('status_update', {'text': 'Recording...', 'color': 'red', 'rec_info': 'Waiting for data...'})

    elif cmd == 'rec_stop':
        node.recording = False
        name = node.save_recording(data.get('name', ''))
        if name: emit('status_update', {'text': f'Saved: {name}', 'color': 'lime', 'rec_info': 'Ready'})
        else: emit('status_update', {'text': 'Save Failed', 'color': 'orange', 'rec_info': 'No Data'})

    elif cmd == 'rec_discard':
        node.recording = False
        node.episode_data = []
        emit('status_update', {'text': 'Discarded', 'color': 'orange', 'rec_info': 'Ready'})


# ... (Keep all imports and the HTML_TEMPLATE as they are) ...

# ... (Keep the PandaBackend class and Flask routes as they are) ...

# --- GLOBAL REFERENCE ---
node = None 

def ros_spin_thread():
    # Spin the node indefinitely
    rclpy.spin(node)

def main(args=None):
    global node
    rclpy.init(args=args)

    # 1. Initialize the Node
    node = PandaBackend()

    # 2. Start the ROS Spin Thread
    t = threading.Thread(target=ros_spin_thread, daemon=True)
    t.start()
    
    print("=============================================")
    print("WEB CONTROL RUNNING AT: http://0.0.0.0:5000")
    print("=============================================")

    # 3. Run the Web Server (Blocking)
    try:
        socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True, use_reloader=False)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup when server stops
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()