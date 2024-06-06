from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import random
import eventlet
import time

eventlet.monkey_patch()

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)

# Array indicating the status of each light: True is green, False is red
lights_status = [random.choice([True, False]) for _ in range(10)]

@app.route('/')
def index():
    return render_template('index.html', lights=lights_status)

@app.route('/receive_data', methods=['POST'])
def receive_data():
    data = request.json
    # Process the data as needed
    print(f"Received data: {data}")
    return jsonify({"status": "success", "received_data": data})

@app.route('/send_data', methods=['GET'])
def send_data():
    data = {"message": "Hello from Flask!"}
    return jsonify(data)

def send_coordinates():
    lat = 50.284833
    lng = 18.976528
    while True:
        lat += random.uniform(0.0001, 0.0002)
        lng += random.uniform(0.0001, 0.0002)
        socketio.emit('drone_coords', {'lat': lat, 'lng': lng}, broadcast=True)
        time.sleep(1)

@socketio.on('connect')
def handle_connect():
    print('Client connected')

if __name__ == '__main__':
    socketio.start_background_task(target=send_coordinates)
    socketio.run(app, host='0.0.0.0', debug=True)