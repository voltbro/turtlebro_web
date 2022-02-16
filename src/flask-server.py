#! /usr/bin/python3

import rospy
from rosgraph.rosenv import get_master_uri
from urllib.parse import urlparse

import os,signal,sys
from flask import Flask, send_from_directory, send_file, request, jsonify, render_template
import rosgraph.masterapi

rospy.init_node('webserver', anonymous=True)

port = rospy.get_param('~port', 8080)
host = rospy.get_param('~host', '0.0.0.0')
www_path = rospy.get_param('~path','../web')
debug = rospy.get_param('~debug', False)

master = rosgraph.masterapi.Master('/rostopic')
app = Flask(__name__, static_folder=www_path + '/static', template_folder=www_path)

if debug:
    app.debug = True
    app.jinja_env.auto_reload = True
    app.config['TEMPLATES_AUTO_RELOAD'] = True

def set_exit_handler(func):
    signal.signal(signal.SIGTERM, func)
    signal.signal(signal.SIGINT, func)

def on_exit(sig, func=None):
    print ("Exit handler triggered")
    sys.exit(1)


def get_video_topics():
    topics = []
    for topic in master.getPublishedTopics(''):
        if topic[1] == 'sensor_msgs/CompressedImage':
            topics.append(topic[0].replace('/compressed',''))

    return topics        

@app.route('/')
def serve_index():
    # host_params = request.host.split(":")
    # ros_host_params = urlparse(get_master_uri())
    ip_address = request.host.split(':')[0]
    return render_template('index.html', 
            # ros_host = ros_host_params.hostname,
            ros_host = ip_address,
            ros_robot = os.uname()[1],
            video_topics = get_video_topics()
            )

set_exit_handler(on_exit)
rospy.loginfo('Start WebServer on {}:{}'.format(host, port))

app.run(host=host, port=port, threaded=True)
