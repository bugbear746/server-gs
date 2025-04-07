from flask import Flask, request, jsonify
import time
import os

app = Flask(__name__)

# Store latest data
telemetry_data = {}
video_buffer = b""
command = ""

@app.route('/telemetry', methods=['POST'])
def receive_telemetry():
    global telemetry_data
    telemetry_data = request.get_json()
    return jsonify({"status": "received"}), 200

@app.route('/video', methods=['POST'])
def receive_video():
    global video_buffer
    video_chunk = request.files['file'].read()
    video_buffer += video_chunk
    if b'\xff\xd9' in video_buffer:  # End of JPEG
        with open('latest_video.jpg', 'wb') as f:
            f.write(video_buffer[:video_buffer.index(b'\xff\xd9') + 2])
        video_buffer = video_buffer[video_buffer.index(b'\xff\xd9') + 2:]
    return jsonify({"status": "received"}), 200

@app.route('/command', methods=['GET'])
def get_command():
    global command
    return jsonify({"command": command})

@app.route('/command', methods=['POST'])
def set_command():
    global command
    command = request.get_json().get("command", "") or request.get_json().get("response", "")
    return jsonify({"status": "set"}), 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)