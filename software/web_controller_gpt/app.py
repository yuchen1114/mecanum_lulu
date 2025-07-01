from flask import Flask, render_template, request
from socket_client import send_command

app = Flask(__name__)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/control", methods=["POST"])
def control():
    cmd = request.form["cmd"]
    response = send_command(cmd)
    return f"已送出：{cmd}，回應：{response}"

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
