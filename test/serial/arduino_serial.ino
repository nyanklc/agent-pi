byte received_count = 0;
bool blink_state = 0;

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  float values[3] = {0, 0, 0};
  // RPi sends 9 characters
  if (Serial.available() >= 0) {
    String inputString = Serial.readStringUntil('\n');

    inputString.trim();

    int i = 0;
    char* p = strtok((char*)inputString.c_str(), " ");
    while (p != NULL && i < 3) {
      values[i++] = atof(p);
      p = strtok(NULL, " ");
    }
    received_count++;
  }

  // READ SPEED VALUES
  double linear_speed = values[0]; // m/s
  double angular_speed = values[1]; // rad/s
  double camera_angular_speed = values[2]; // rad/s

  char response[5] = {'a', 'r', 'd', (char)received_count, '\n'};
  Serial.print(response);
  blink_state = !blink_state;
  digitalWrite(LED_BUILTIN, blink_state ? HIGH : LOW);
  Serial.flush();

  delay(1);
}
