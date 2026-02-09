#!/usr/bin/env python3

import os
import json
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String  # <--- Added for operating_mode

from custom_interfaces.action import ArmCommand
from ament_index_python.packages import get_package_share_directory
from flask import Flask, request, jsonify, render_template

from google import genai
from groq import Groq
from dotenv import load_dotenv


# ================= SYSTEM PROMPT =================

SYSTEM_PROMPT = """
You are an expert robotics command generator. Your task is to translate natural language instructions into a strictly formatted JSON object for a 7-DOF robotic arm.

Core Rules:

    Output ONLY valid JSON. No conversational filler, no markdown code blocks, and no explanations.

    Mode Logic:

        joint_control: For specific angular movements.

        cleaning: For requests involving clearing, removing, or tidying the workspace.

        sorting: For requests involving separating, organizing, or categorizing objects.

    The move Array: Must contain exactly 7 elements.

        Use -402 to signify "no change" or "null" for specific joints.

        For cleaning and sorting modes, default the array to [-402, -402, -402, -402, -402, -402, -402] unless specific start/end angles are mentioned.

    The gripper Array: 1 (close), 0 (open), -1 (no change).

JSON Structure:
JSON

{
  "mode": "string",
  "move": [j1, j2, j3, j4, j5, j6, j7],
  "gripper": [g]
}

Instruction Examples:

    Specific Single Joint: "Rotate joint four to 15 degrees." {"mode": "joint_control", "move": [-402, -402, -402, 15, -402, -402, -402], "gripper": [-1]}

    Specific Single Joint: "open gripper" {"mode": "joint_control", "move": [-402, -402, -402, -402, -402, -402, -402], "gripper": [0]}

    Global Movement: "Reset all joints to 0 and open the claw." {"mode": "joint_control", "move": [0, 0, 0, 0, 0, 0, 0], "gripper": [0]}

    Complex Multi-Joint: "Set j1 to 45, j2 to -10, and j7 to 90." {"mode": "joint_control", "move": [45, -10, -402, -402, -402, -402, 90], "gripper": [-1]}

    Cleaning Task: "Clear all the blocks off the table. or move all cubes to one bin. or trash all blocks" {"mode": "cleaning", "move": [-402, -402, -402, -402, -402, -402, -402], "gripper": [-1]}

    Sorting Task: "Separate the coloured blocks. or move blocks to respected bin" {"mode": "sorting", "move": [-402, -402, -402, -402, -402, -402, -402], "gripper": [-1]}
user input:

"""


# ================= ROS + FLASK NODE =================

class GeminiWebNode(Node):

    def __init__(self):
        super().__init__('gemini_web_node')


        load_dotenv()

        # ---------- ROS Action Client ----------
        self.action_client = ActionClient(
            self, ArmCommand, '/arm_command'
        )

        self.current_mode = "joint_control"
        self.mode_pub = self.create_publisher(String, 'operating_mode', 10)
        self.timer = self.create_timer(0.1, self.publish_mode_callback)

        # ---------- LLM Clients ----------
        self.gemini_client = genai.Client(
            api_key=os.getenv("GEMINI_API_KEY")
        )

        self.groq_client = Groq(
            api_key=os.getenv("GROQ_API_KEY")
        )

        # ---------- Flask Setup ----------
        pkg_share = get_package_share_directory('gemini_caller')
        template_dir = os.path.join(pkg_share, 'templates')
        static_dir = os.path.join(pkg_share, 'static')

        self.app = Flask(
            __name__,
            template_folder=template_dir,
            static_folder=static_dir
        )

        self.register_routes()
        self.get_logger().info("✅ Gemini + Groq Web Node ready")


    def publish_mode_callback(self):
        msg = String()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)
    # ================= ROUTES =================

    def register_routes(self):

        @self.app.route("/", methods=["GET"])
        def index():
            return render_template("index.html")

        # ----- CHAT FRONTEND ROUTE -----
        @self.app.route("/generate", methods=["POST"])
        def generate():
            data = request.get_json()
            user_text = data.get("prompt", "")
            model_choice = data.get("model", "groq")

            try:
                # ---- LLM call ----
                if model_choice == "groq":
                    raw = self.call_groq(user_text)
                else:
                    raw = self.call_gemini(user_text)

                parsed = json.loads(raw)
                self.current_mode = parsed.get('mode', 'joint_control')
            
            except Exception as e:
                return jsonify({
                    "response": f"❌ LLM parsing error: {str(e)}"
                })

            # ---- ROS Action ----
            if not self.action_client.wait_for_server(timeout_sec=2.0):
                return jsonify({
                    "response": "❌ Robot action server not available"
                })

            goal = ArmCommand.Goal()
            goal.json_command = json.dumps(parsed)
            self.action_client.send_goal_async(goal)

            # ---- Chat-friendly response ----
            pretty_response = (
                "✅ Command sent to robot\n\n"
                f"Mode: {parsed['mode']}\n"
                f"Joints: {parsed['move']}\n"
                f"Gripper: {parsed['gripper']}"
            )

            return jsonify({"response": pretty_response})

        # ----- RAW JSON API (OPTIONAL) -----
        @self.app.route("/command", methods=["POST"])
        def command():
            data = request.get_json()
            user_text = data.get("text", "")
            model_choice = data.get("model", "groq")

            try:
                if model_choice == "groq":
                    raw = self.call_groq(user_text)
                else:
                    raw = self.call_gemini(user_text)

                parsed = json.loads(raw)

            except Exception as e:
                return jsonify({"error": str(e)}), 500

            if not self.action_client.wait_for_server(timeout_sec=2.0):
                return jsonify({"error": "Action server not available"}), 500

            goal = ArmCommand.Goal()
            goal.json_command = json.dumps(parsed)
            self.action_client.send_goal_async(goal)

            return jsonify({
                "input": user_text,
                "output": parsed,
                "status": "Action sent"
            })

    # ================= LLM CALLS =================

    def call_groq(self, user_text: str) -> str:
        completion = self.groq_client.chat.completions.create(
            model="llama-3.1-8b-instant",
            messages=[
                {
                    "role": "user",
                    "content": SYSTEM_PROMPT + "\nUser command: " + user_text
                }
            ],
        )
        return completion.choices[0].message.content.strip()

    def call_gemini(self, user_text: str) -> str:
        response = self.gemini_client.models.generate_content(
            model="gemini-1.0-pro",
            contents=SYSTEM_PROMPT + "\nUser command: " + user_text
        )
        return response.text.strip()


# ================= MAIN =================

def main():
    rclpy.init()
    node = GeminiWebNode()

    flask_thread = threading.Thread(
        target=lambda: node.app.run(
            host="0.0.0.0",
            port=5000,
            debug=False
        ),
        daemon=True
    )
    flask_thread.start()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()