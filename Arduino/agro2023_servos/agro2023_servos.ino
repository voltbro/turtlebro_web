#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

#define SERVO_44_INIT 90
#define SERVO_45_INIT 50


class NewHardware : public ArduinoHardware
{
  public:
  NewHardware():ArduinoHardware(&Serial1, 115200){};
};

ros::NodeHandle_<NewHardware>  nh;

Servo servo44; //gorizont
Servo servo45; //vertical

void servo44_cb( const std_msgs::UInt16& cmd_msg){
  servo44.write(cmd_msg.data); //set servo angle, should be from 0-180  
}

void servo45_cb( const std_msgs::UInt16& cmd_msg){
  servo45.write(cmd_msg.data); //set servo angle, should be from 0-180  
}

void led_cb( const std_msgs::UInt16& cmd_msg){
  
  if (cmd_msg.data == 0){
    digitalWrite(13, LOW);
  }
  
  if (cmd_msg.data == 1){
    digitalWrite(13, HIGH);
  } 
}


ros::Subscriber<std_msgs::UInt16> sub44("servo44", servo44_cb);
ros::Subscriber<std_msgs::UInt16> sub45("servo45", servo45_cb);

ros::Subscriber<std_msgs::UInt16> sub_led("led", led_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub44);
  nh.subscribe(sub45);
  nh.subscribe(sub_led);
    
  servo44.attach(44);
  servo44.write(SERVO_44_INIT);
  
  servo45.attach(45);   
  servo45.write(SERVO_45_INIT);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
