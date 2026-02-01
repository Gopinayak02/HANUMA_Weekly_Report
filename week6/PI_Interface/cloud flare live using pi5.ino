🔹 Step 1: Update Raspberry Pi

sudo apt update && sudo apt upgrade -y

🔹 Step 2: Install Cloudflared
wget -q https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm64.deb
sudo dpkg -i cloudflared-linux-arm64.deb

Verify installation:
cloudflared --version

🔹 Step 3: Authenticate Cloudflare
cloudflared tunnel login

➡ This opens a browser
➡ Select your Cloudflare account
➡ Choose your domain
➡ Certificate is saved automatically

🔹 Step 4: Create a Tunnel
cloudflared tunnel create pi5-stream

You’ll see output like:
Tunnel credentials written to /home/pi/.cloudflared/xxxxxxxx.json

🔹 Step 5: Configure Tunnel
Create config file:
mkdir -p ~/.cloudflared
nano ~/.cloudflared/config.yml

Paste this (EDIT DOMAIN + PORT):
tunnel: pi5-stream
credentials-file: /home/pi/.cloudflared/xxxxxxxx.json
ingress:
  - hostname: pi5.yourdomain.com
    service: http://localhost:5000
  - service: http_status:404

5000 → your local streaming server port
Example: Flask/OpenCV stream runs on http://localhost:5000

🔹 Step 6: Add DNS Route
cloudflared tunnel route dns pi5-stream pi5.yourdomain.com

🔹 Step 7: Run Tunnel (Manual)
cloudflared tunnel run pi5-stream
Now open:
https://pi5.yourdomain.com

Your Raspberry Pi 5 live stream is public & secure

🔹 Step 8: Run Tunnel as a Service (Auto-Start)
sudo cloudflared service install
sudo systemctl enable cloudflared
sudo systemctl start cloudflared

Check status:
systemctl status cloudflared

🔹 Example: Flask Live Streaming App (Optional)

from flask import Flask, Response
import cv2
app = Flask(__name__)
camera = cv2.VideoCapture(0)
def gen():
    while True:
        ret, frame = camera.read()
        if not ret:
            break
        _, jpeg = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
@app.route('/video')
def video():
    return Response(gen(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')
app.run(host='0.0.0.0', port=5000)
                       
🔹 Security & Notes
 No port forwarding
 No static IP needed
 HTTPS by default
 Works behind NAT / college Wi-Fi
 Tunnel depends on internet stability
