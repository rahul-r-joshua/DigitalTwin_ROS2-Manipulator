#include <Servo.h>

Servo servo8;
Servo servo9;
Servo servo10;
Servo servo11;

const int PIN_SERVO_8 = 8;
const int PIN_SERVO_9 = 9;
const int PIN_SERVO_10 = 10;
const int PIN_SERVO_11 = 11;

void setup() {
  servo8.attach(PIN_SERVO_8);
  servo9.attach(PIN_SERVO_9);
  servo10.attach(PIN_SERVO_10);
  servo11.attach(PIN_SERVO_11);
  
  Serial.begin(9600);

  Serial.println("--- Four Servo Control Initialized ---");
  Serial.println("Enter commands in the format: [PIN]:[ANGLE] (e.g., 9:120)");
  Serial.println("Pins available: 8, 9, 10, 11. Angle range: 0 to 180.");

  servo8.write(90);
  servo9.write(90);
  servo10.write(90);
  servo11.write(90);
}

void loop() {
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    inputString.trim(); 

    int delimiterIndex = inputString.indexOf(':');

    if (delimiterIndex > 0) {
      String pinString = inputString.substring(0, delimiterIndex);
      int targetPin = pinString.toInt();

      String angleString = inputString.substring(delimiterIndex + 1);
      int targetAngle = angleString.toInt();

      if (targetAngle >= 0 && targetAngle <= 180) {

        if (targetPin == PIN_SERVO_8) {
          servo8.write(targetAngle);
          Serial.print("Servo on Pin 8 moved to: ");
          Serial.println(targetAngle);
        } else if (targetPin == PIN_SERVO_9) {
          servo9.write(targetAngle);
          Serial.print("Servo on Pin 9 moved to: ");
          Serial.println(targetAngle);
        } else if (targetPin == PIN_SERVO_10) {
          servo10.write(targetAngle);
          Serial.print("Servo on Pin 10 moved to: ");
          Serial.println(targetAngle);
        } else if (targetPin == PIN_SERVO_11) {
          servo11.write(targetAngle);
          Serial.print("Servo on Pin 11 moved to: ");
          Serial.println(targetAngle);
        } else {
          Serial.print("Error: Invalid pin number (");
          Serial.print(targetPin);
          Serial.println("). Use 8, 9, 10 or 11.");
        }
        
      } else {
        Serial.print("Error: Angle ");
        Serial.print(targetAngle);
        Serial.println(" is out of range (0-180).");
      }
    } else {
      Serial.print("Error: Invalid command format. Expected [PIN]:[ANGLE], got '");
      Serial.print(inputString);
      Serial.println("'");
    }
  }
}