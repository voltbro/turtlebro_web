/*
 * Серво полива (44) верхний максимум 155
 * Серво полива (44) нижний максиум 50
 * Серво полива поворот (45) крайнее правое 30
 * Серво полива поворот (45) крайнее левое 180
 * Серво верхней камеры (9) верхний максимум 
 * Серво верхней камеры (9) нижний максимум 
 * Серво верхней камеры (10) левый максимум 
 * Серво верхней камеры (10) провый максимум 
 * 
 */

#include <Servo.h>
#include <ros.h>
#include "std_msgs/Int16.h"

//arduino to ROS connect init
class NewHardware : public ArduinoHardware {
public:
  NewHardware()
    : ArduinoHardware(&Serial1, 115200){};
};

//ROS node init
ros::NodeHandle_<NewHardware> nh;

//Servos and pins init

int poliv_ud_angle;
int camera_ud_angle;

int poliv_u_d_init = 10;

int camera_u_d_init = 60;
int camera_l_r_init = 90;

Servo Poliv_up_down;

Servo Camera_up_down;
Servo Camera_left_right;

void flushMotor(int m2) { 
  if (m2 == 1) {
    // Увеличиваем значение, но не более 150
    if (poliv_ud_angle < 150) {
      poliv_ud_angle++;
    }
    poliv_ud_angle = min(poliv_ud_angle, 150);
  } else if (m2 == 2) {
    // Уменьшаем значение, но не менее 10
    if (poliv_ud_angle > 10) {
      poliv_ud_angle--;
    }
    poliv_ud_angle = max(poliv_ud_angle, 10);
  }
  // Публикуем данные
  Poliv_up_down.write(poliv_ud_angle);
}


void CbCameraUD(const std_msgs::Int16& angle) {
  Camera_up_down.write(angle.data);
}
ros::Subscriber<std_msgs::Int16> subCameraUD("servo45", &CbCameraUD);

void CbCameraLR(const std_msgs::Int16& angle) {
  Camera_left_right.write(angle.data);
}
ros::Subscriber<std_msgs::Int16> subCameraLR("servo44", &CbCameraLR);


void CbFlushPump(const std_msgs::Int16& cmnd) {
  flushMotor(cmnd.data);
}
ros::Subscriber<std_msgs::Int16> subFlushPump("flush_pump", &CbFlushPump);


void setup() {
  //ROS nodes init
  nh.initNode();

  nh.subscribe(subCameraUD);
  nh.subscribe(subCameraLR);
  nh.subscribe(subFlushPump);


  // Rotate to initial servos angles
  Poliv_up_down.attach(44);
  Poliv_up_down.write(poliv_u_d_init);


  Camera_up_down.attach(1);
  Camera_up_down.write(camera_u_d_init);
  Camera_left_right.attach(3);
  Camera_left_right.write(camera_l_r_init);

  // Water pump inits
}

void loop() {
  //ROS node spin
  nh.spinOnce();
  delay(1);
}