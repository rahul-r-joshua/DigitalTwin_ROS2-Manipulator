#include <Servo.h>

#define SERVO_BASE_PIN 8
#define SERVO_SHOULDER_PIN 9
#define SERVO_ELBOW_PIN 10
#define SERVO_GRIPPER_PIN 11

#define BASE_START 90
#define SHOULDER_START 90
#define ELBOW_START 107
#define GRIPPER_START 90

#define BASE_MIN 0
#define BASE_MAX 180

#define SHOULDER_MIN 0
#define SHOULDER_MAX 180

#define ELBOW_MIN 107
#define ELBOW_MAX 180

#define GRIPPER_MIN 90
#define GRIPPER_MAX 180

Servo base, shoulder, elbow, gripper;

char buffer[32];
uint8_t bufferIndex = 0;

void setup() {

  base.attach(SERVO_BASE_PIN);
  shoulder.attach(SERVO_SHOULDER_PIN);
  elbow.attach(SERVO_ELBOW_PIN);
  gripper.attach(SERVO_GRIPPER_PIN);

  base.write(BASE_START);
  shoulder.write(SHOULDER_START);
  elbow.write(ELBOW_START);
  gripper.write(GRIPPER_START);

  Serial.begin(115200);
  Serial.setTimeout(5);

  delay(100);
  Serial.println("ArduinoBot Live Controller Ready!");
  Serial.println("Waiting for MoveIt commands...");
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();

    if (bufferIndex < sizeof(buffer) - 1) {
      buffer[bufferIndex++] = c;
      buffer[bufferIndex] = '\0';

      if (c == ',') {
        processCommand();
        bufferIndex = 0;  
      }
    } else {
      bufferIndex = 0;
    }
  }
}

void processCommand() {

  
  char* ptr = buffer;
  
  while (*ptr != '\0') {
    char servo = *ptr;
    ptr++;

    int value = 0;
    while (*ptr >= '0' && *ptr <= '9') {
      value = value * 10 + (*ptr - '0');
      ptr++;
    }

    switch(servo) {
      case 'b': 
        value = constrain(value, BASE_MIN, BASE_MAX);
        base.write(value);
        break;
        
      case 's': 
        value = constrain(value, SHOULDER_MIN, SHOULDER_MAX);
        shoulder.write(value);
        break;
        
      case 'e':  
        value = constrain(value, ELBOW_MIN, ELBOW_MAX);
        elbow.write(value);
        break;
        
      case 'g': 
        value = constrain(value, GRIPPER_MIN, GRIPPER_MAX);
        gripper.write(value);
        break;
    }

    if (*ptr == ',') ptr++;
  }
}
