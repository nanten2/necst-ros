#!/usr/bin/env python3
from flask import *

app = Flask(__name__, static_folder="../static")


@app.route("/")
def index():
    with open("../static/index5.html") as f:
        html = f.read()
    return render_template("index.html", in_html=html)


if __name__ == "__main__":
    app.run(host="0.0.0.0", debug=True, port=12321, threaded=True)
