#include <Stepper.h>

const int steps_per_rev = 200;
const int camera_stepper_setspeed = 30;

// PINS
const int camera_stepper_1 = 8;
const int camera_stepper_2 = 10;
const int camera_stepper_3 = 11;
const int camera_stepper_4 = 12;

const int left_motor_1 = 3;
const int left_motor_2 = 5;
const int right_motor_1 = 6;
const int right_motor_2 = 9;

Stepper camera_stepper(steps_per_rev, camera_stepper_1, camera_stepper_2, camera_stepper_3, camera_stepper_4);

void drive_motors(int left_s, int right_s)
{
  // fix directions
  left_s *= -1;
  right_s *= -1;

  if (left_s > 0)
  {
    analogWrite(left_motor_1, left_s);
    analogWrite(left_motor_2, 0);
  }
  else if (left_s < 0)
  {
    analogWrite(left_motor_1, 0);
    analogWrite(left_motor_2, left_s);
  }
  else
  {
    analogWrite(left_motor_1, 0);
    analogWrite(left_motor_2, 0);
  }

  if (right_s > 0)
  {
    analogWrite(right_motor_1, right_s);
    analogWrite(right_motor_2, 0);
  }
  else if (right_s < 0)
  {
    analogWrite(right_motor_1, 0);
    analogWrite(right_motor_2, right_s);
  }
  else
  {
    analogWrite(right_motor_1, 0);
    analogWrite(right_motor_2, 0);
  }
}

void drive_camera_stepper(int camera_s)
{
  if (camera_s != 0)
  {
    camera_stepper.step(camera_s);
  }
  else
  {
    digitalWrite(camera_stepper_1, LOW);
    digitalWrite(camera_stepper_2, LOW);
    digitalWrite(camera_stepper_3, LOW);
    digitalWrite(camera_stepper_4, LOW);
  }
}

void setup()
{
  Serial.begin(9600);

  // pins
  pinMode(left_motor_1, OUTPUT);
  pinMode(left_motor_2, OUTPUT);
  pinMode(right_motor_1, OUTPUT);
  pinMode(right_motor_2, OUTPUT);

  // stepper
  camera_stepper.setSpeed(camera_stepper_setspeed);
}

void loop()
{
  float values[3] = {0, 0, 0};
  // RPi sends 9 characters
  if (Serial.available() > 0)
  {
    String inputString = Serial.readStringUntil('\n');


    inputString.trim();

    int i = 0;
    char* p = strtok((char*)inputString.c_str(), " ");
    while (p != NULL && i < 3)
    {
      values[i++] = atof(p);
      p = strtok(NULL, " ");
    }

  }
  else
  {
    delay(10);
    return;
  }

  // READ SPEED VALUES
  double left_s = values[0]; // m/s
  double right_s = values[1]; // rad/s
  double camera_s = values[2]; // rad/s

  // drive motors
  drive_motors(left_s, right_s);

  // drive camera stepper
  drive_camera_stepper(camera_s);
}
