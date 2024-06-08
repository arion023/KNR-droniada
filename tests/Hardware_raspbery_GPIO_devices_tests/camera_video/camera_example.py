import ffmpeg

if __name__ == "__main__":
    import ffmpeg
import numpy as np
from PIL import Image
import io

# Stream URL
stream_url = 'http://localhost:8080/?action=stream'

# Set up FFmpeg command to capture one frame
out, _ = (
    ffmpeg
    .input(stream_url)
    .output('pipe:', vframes=1, format='image2', vcodec='mjpeg')
    .run(capture_stdout=True, capture_stderr=True)
)

# Convert the output to a numpy array
image_np = np.frombuffer(out, np.uint8)

# Load the image with PIL
image = Image.open(io.BytesIO(image_np))

# Display or process the image as needed
#image.show()  # This will display the image

image.save('output.jpg')