#! /usr/bin/python
import rospy
import os,signal,sys
from flask import Flask, send_from_directory, send_file, request, jsonify, render_template

rospy.init_node('webserver', anonymous=True)

port = rospy.get_param('~port', 8080)
host = rospy.get_param('~host', '0.0.0.0')
www_path = rospy.get_param('~path','../web')

app = Flask(__name__, static_folder=www_path + '/static', template_folder=www_path)

def set_exit_handler(func):
    signal.signal(signal.SIGTERM, func)
    signal.signal(signal.SIGINT, func)

def on_exit(sig, func=None):
    print ("Exit handler triggered")
    sys.exit(1)

@app.route('/')
def serve_index():
    host_params = request.host.split(":")
    # return render_template('index.html', navibro_host='navibro.local')
    return render_template('index.html', navibro_host=host_params[0])

set_exit_handler(on_exit)
rospy.loginfo('Start WebServer on {}:{}'.format(host, port))

app.run(host=host, port=port, threaded=True)
