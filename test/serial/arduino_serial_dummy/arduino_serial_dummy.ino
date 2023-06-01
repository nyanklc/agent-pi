#include <Stepper.h>

const int steps_per_rev = 200;
const int camera_stepper_setspeed = 60; // ?

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
    if (left_s > 255)
      left_s = 255;
    analogWrite(left_motor_1, left_s);
    analogWrite(left_motor_2, 0);
  }
  else if (left_s < 0)
  {
    if (left_s < -255)
      left_s = -255;
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
    if (right_s > 255)
      right_s = 255;
    analogWrite(right_motor_1, right_s);
    analogWrite(right_motor_2, 0);
  }
  else if (right_s < 0)
  {
    if (right_s < -255)
      right_s = -255;
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
  camera_stepper.step(camera_s);
}

void send_response(char ch)
{
  Serial.write('a');
}

void setup()
{
  Serial.begin(9600);

  // stepper
  camera_stepper.setSpeed(camera_stepper_setspeed);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
  // drive camera stepper
  drive_camera_stepper(200);
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

}
