from flask import Flask, render_template, request, jsonify
import random

app = Flask(__name__)

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

# Endpoint to send data
@app.route('/send_data', methods=['GET'])
def send_data():
    data = {"message": "Hello from Flask!"}
    return jsonify(data)

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)
