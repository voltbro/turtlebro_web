/**
 * @author Russell Toris - rctoris@wpi.edu
 */

var KEYBOARDTELEOP = KEYBOARDTELEOP || {
  REVISION : '0.4.0-SNAPSHOT'
};

/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * Manages connection to the server and all interactions with ROS.
 *
 * Emits the following events:
 *   * 'change' - emitted with a change in speed occurs
 *
 * @constructor
 * @param options - possible keys include:
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic (optional) - the Twist topic to publish to, like '/cmd_vel'
 *   * throttle (optional) - a constant throttle for the speed
 */
KEYBOARDTELEOP.Teleop = function(options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/cmd_vel';
  // permanent throttle
  var throttle = options.throttle || 1.0;

  // used to externally throttle the speed (e.g., from a slider)
  this.scale = 1.0;

  // linear x and y movement and angular z movement
  var x = 0;
  var y = 0;
  var z = 0;
  var poliv_ud_pose = 140;  
  var poliv_lr_pose = 90;  
  var pump_state = 0;  

  var cmdVel = new ROSLIB.Topic({
    ros : ros,
    name : topic,
    messageType : 'geometry_msgs/Twist'
  });

  var poliv_ud_topic = new ROSLIB.Topic({
    ros : ros,
    name : "poliv_ud",
    messageType : 'std_msgs/Int16'
  });  

  var poliv_lr_topic = new ROSLIB.Topic({
    ros : ros,
    name : "poliv_lr",
    messageType : 'std_msgs/Int16'
  });    

  var pump_topic = new ROSLIB.Topic({
    ros : ros,
    name : "flush_pump",
    messageType : 'std_msgs/Int16'
  });    

  poliv_lr_topic.publish(new ROSLIB.Message({data: poliv_lr_pose}));
  poliv_ud_topic.publish(new ROSLIB.Message({data: poliv_ud_pose}));

  // sets up a key listener on the page used for keyboard teleoperation
  var handleKey = function(keyCode, keyDown) {
    // used to check for changes in speed
    var oldX = x;
    var oldY = y;
    var oldZ = z;

    var pub = true;

    var speed = 0;
    // throttle the speed by the slider and throttle constant
    if (keyDown === true) {
      speed = throttle * that.scale;
    }
    // check which key was pressed
    switch (keyCode) {

      case 73://i controll up
        if (poliv_ud_pose > 100) poliv_ud_pose -=1;
        poliv_ud_topic.publish(new ROSLIB.Message({data: poliv_ud_pose}));
        break;

      case 75://k controll down
        if (poliv_ud_pose < 155) poliv_ud_pose +=1;
        poliv_ud_topic.publish(new ROSLIB.Message({data: poliv_ud_pose}));      

        break;

      case 76://l controll right
        if (poliv_lr_pose > 40) poliv_lr_pose -=1;
        poliv_lr_topic.publish(new ROSLIB.Message({data: poliv_lr_pose}));
        break;

      case 74://j controll left
        if (poliv_lr_pose < 180) poliv_lr_pose +=1;
        poliv_lr_topic.publish(new ROSLIB.Message({data: poliv_lr_pose}));      
        break;

      case 79://o pump off
        pump_topic.publish(new ROSLIB.Message({data: 0}));    
        break;  

      case 80://o pump on
        pump_topic.publish(new ROSLIB.Message({data: 1}));    
        break;          
                
      case 65:
        // turn left
        z = 0.9 * speed;
        break;
      case 87:
        // up
        x = 0.25 * speed;
        break;
      case 68:
        // turn right
        z = -0.9 * speed;
        break;
      case 83:
        // down
        x = -0.25 * speed;
        break;
      case 69:
        // strafe right
        y = -0.5 * speed;
        break;
      case 81:
        // strafe left
        y = 0.5 * speed;
        break;
      default:
        pub = false;
    }

    // publish the command
    if (pub === true) {
      
      var twist = new ROSLIB.Message({
        angular : {
          x : 0,
          y : 0,
          z : z
        },
        linear : {
          x : x,
          y : y,
          z : z
        }
      });
      cmdVel.publish(twist);



      // check for changes
      if (oldX !== x || oldY !== y || oldZ !== z) {
        that.emit('change', twist);
      }
    }
  };

  // handle the key
  var body = document.getElementsByTagName('body')[0];
  body.addEventListener('keydown', function(e) {
    handleKey(e.keyCode, true);
  }, false);
  body.addEventListener('keyup', function(e) {
    handleKey(e.keyCode, false);
  }, false);
};
KEYBOARDTELEOP.Teleop.prototype.__proto__ = EventEmitter2.prototype;
