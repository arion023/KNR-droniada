from flask import Flask, render_template
import random

app = Flask(__name__)

# Array indicating the status of each light: True is green, False is red
lights_status = [random.choice([True, False]) for _ in range(10)]

@app.route('/')
def index():
    return render_template('index.html', lights=lights_status)

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)
