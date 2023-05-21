const int StepX = 2;
const int DirX = 5;

void setup() {
  pinMode(StepX,OUTPUT);
  pinMode(DirX,OUTPUT);
}

void loop() {
    digitalWrite(DirX, HIGH);

    for (int x = 0; x<200; x++) {
        digitalWrite(StepX,HIGH);
        delayMicroseconds(500);
        digitalWrite(StepX,LOW); 
        delayMicroseconds(500);
    }

}
