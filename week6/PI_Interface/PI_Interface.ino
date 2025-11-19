from flask import Flask, Response, request, redirect, render_template_string, session
from flask_session import Session
import cv2

app = Flask(_name_)

# ---------- SESSION CONFIG ----------
app.config["SECRET_KEY"] = "bindu123"  # change this
app.config["SESSION_TYPE"] = "filesystem"
Session(app)

# ---------- PASSWORD ----------
LOGIN_PASSWORD = "gopi123"   # <-- change your password here

# ---------- CAMERA ----------
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

# ---------- LOGIN PAGE ----------
login_page = """
<!DOCTYPE html>
<html>
<head>
<title>Login</title>
</head>
<body style="font-family: Arial; text-align:center;">
<h2>🔐 Secure Stream Login</h2>
<form method="POST">
    <input type="password" name="password" placeholder="Enter Password" style="padding:10px; width:200px;">
    <br><br>
    <button type="submit" style="padding:10px 20px;">Login</button>
</form>
</body>
</html>
"""

# ---------- STREAM PAGE ----------
stream_page = """
<!DOCTYPE html>
<html>
<head>
<title>Live Stream</title>
</head>
<body style="text-align:center;">
<h2>📡 Public Live Stream (Protected)</h2>
<img src="/video_feed" width="70%">
</body>
</html>
"""

# ---------- AUTH CHECK ----------
def is_logged_in():
    return session.get("logged_in", False)

# ---------- ROUTES ----------
@app.route("/", methods=["GET", "POST"])
def login():
    if request.method == "POST":
        pwd = request.form.get("password")
        if pwd == LOGIN_PASSWORD:
            session["logged_in"] = True
            return redirect("/stream")
        else:
            return "<h3>❌ Wrong password</h3>" + login_page

    return login_page

@app.route("/stream")
def stream():
    if not is_logged_in():
        return redirect("/")
    return render_template_string(stream_page)

@app.route("/video_feed")
def video_feed():
    if not is_logged_in():
        return redirect("/")
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# ---------- FRAME GENERATOR ----------
def gen_frames():
    while True:
        success, frame = cap.read()
        if not success:
            break
        _, buffer = cv2.imencode(".jpg", frame)
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n")

# ---------- START SERVER ----------
if _name_ == "_main_":
    app.run(host="0.0.0.0", port=5000, threaded=True)