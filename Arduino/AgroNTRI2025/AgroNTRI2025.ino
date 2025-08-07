#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int16.h>

// === Объявление сервоприводов ===
Servo Lift_servo;             // Серво для подъёма/опускания захвата
Servo Camera_up_down;         // Серво для наклона камеры (вверх/вниз)
Servo Camera_left_right;      // Серво для поворота камеры (влево/вправо)

// === Начальные углы ===
int currentAngle = 10;        // Текущий угол подъёмного серво (граница: 10–150)
int camera_u_d_init = 60;     // Начальный угол наклона камеры
int camera_l_r_init = 90;     // Начальный угол поворота камеры

// === Флаги управления подъемом ===
bool movingUp = false;        // Подъём захвата активен
bool movingDown = false;      // Опускание захвата активно

// === Настройка ROS через Serial1 ===
class NewHardware : public ArduinoHardware {
public:
  NewHardware() : ArduinoHardware(&Serial1, 115200) {}
};

ros::NodeHandle_<NewHardware> nh;

// === Callback для управления захватом ===
// msg.data == 1 → двигаться вверх
// msg.data == 2 → двигаться вниз
// msg.data == 0 → остановиться
void gripperLiftCallback(const std_msgs::Int16& msg) {
  int command = msg.data;

  if (command == 1) {
    movingUp = true;
    movingDown = false;
  } else if (command == 2) {
    movingUp = false;
    movingDown = true;
  } else if (command == 0) {
    movingUp = false;
    movingDown = false;
  }
}
ros::Subscriber<std_msgs::Int16> subGripperLift("/gripper_lift", &gripperLiftCallback);

// === Callback для наклона камеры вверх/вниз ===
void CbCameraUD(const std_msgs::Int16& angle) {
  Camera_up_down.write(angle.data);
}
ros::Subscriber<std_msgs::Int16> subCameraUD("/camera_tilt", &CbCameraUD);

// === Callback для поворота камеры влево/вправо ===
void CbCameraLR(const std_msgs::Int16& angle) {
  Camera_left_right.write(angle.data);
}
ros::Subscriber<std_msgs::Int16> subCameraLR("/camera_pan", &CbCameraLR);

void setup() {
  Serial.begin(115200);      // Для отладки (опционально)

  nh.initNode();             // Инициализация ROS-ноды

  // Подписки на топики управления
  nh.subscribe(subGripperLift);
  nh.subscribe(subCameraUD);
  nh.subscribe(subCameraLR);

  // Подключение сервоприводов к пинам
  Lift_servo.attach(44);
  Camera_up_down.attach(45);
  Camera_up_down.write(camera_u_d_init);
  Camera_left_right.attach(46);
  Camera_left_right.write(camera_l_r_init);

  // Установка стартового угла подъёмного серво
  Lift_servo.write(currentAngle);

  delay(1000); // Пауза на инициализацию
}

void loop() {
  nh.spinOnce();  // Обработка входящих ROS-сообщений

  // Управление подъёмом/опусканием захвата
  if (movingDown) {
    if (currentAngle < 150) {
      currentAngle += 2;
      Lift_servo.write(currentAngle);
    }
  } else if (movingUp) {
    if (currentAngle > 10) {
      currentAngle -= 2;
      Lift_servo.write(currentAngle);
    }
  }

  delay(50);  // Задержка между шагами серво
}
