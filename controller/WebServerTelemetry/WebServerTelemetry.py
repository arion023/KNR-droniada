import os
from flask import Flask, request, redirect, url_for, render_template, jsonify
from werkzeug.utils import secure_filename
import random
import time
from flask_socketio import SocketIO

app = Flask(__name__)

# Configure the upload folder
UPLOAD_FOLDER = 'static/uploads/'
ALLOWED_EXTENSIONS = {'png', 'jpg', 'jpeg', 'gif'}
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

# Ensure the upload directory exists
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)

# Array indicating the status of each light: True is green, False is red
lights_status = [random.choice([True, False]) for _ in range(10)]

def allowed_file(filename):
    """Check if a file is allowed based on its extension."""
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS

@app.route('/', methods=['GET', 'POST'])
def index():
    """Handle file uploads and render the index page."""
    if request.method == 'POST':
        if 'file' not in request.files:
            return redirect(request.url)
        file = request.files['file']
        if file.filename == '':
            return redirect(request.url)
        if file and allowed_file(file.filename):
            filename = secure_filename(file.filename)
            file.save(os.path.join(app.config['UPLOAD_FOLDER'], filename))
            return redirect(url_for('index'))

    # List the uploaded files
    uploaded_files = os.listdir(app.config['UPLOAD_FOLDER'])
    return render_template('index.html', lights=lights_status, files=uploaded_files)

@app.route('/receive_data', methods=['POST'])
def receive_data():
    """Receive data via POST and print it (for debugging)."""
    data = request.json
    print(f"Received data: {data}")
    return jsonify({"status": "success", "received_data": data})

@app.route('/send_data', methods=['GET'])
def send_data():
    """Send a simple JSON message."""
    data = {"message": "Hello from Flask!"}
    return jsonify(data)

def send_coordinates():
    """Send random drone coordinates periodically."""
    lat = 50.284833
    lng = 18.976528
    while True:
        lat += random.uniform(-0.0001, 0.0001)
        lng += random.uniform(-0.0001, 0.0001)
        socketio.emit('drone_coords', {'lat': lat, 'lng': lng}, namespace='/')
        time.sleep(1)

@socketio.on('connect')
def handle_connect():
    """Handle client connections."""
    print('Client connected')

if __name__ == '__main__':
    socketio.start_background_task(target=send_coordinates)
    socketio.run(app, host='0.0.0.0', port=5000)
