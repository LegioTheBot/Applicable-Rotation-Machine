#include <Arduino.h>
#include <Servo.h>

enum ControlMode : uint8_t {
  Gamepad,
  MiniArm,
  Exo,
  None
};

const uint8_t pinSpec0 = 12;
const uint8_t pinSpec1 = 13;

class CustomServo : public Servo {
  private:
    float currentAngle;             // Use float for smoother sub-degree steps
    unsigned long lastUpdate;
    const float maxSpeed = 1.5;     // Max degrees per update
    const int deadzone = 30;        // Joystick deadzone around center (514)
    const int updateInterval = 15;  // Update every 15 ms (approx. 66Hz)
    int minAngle = 0;
    int maxAngle = 180;

  public:
    CustomServo() {
      currentAngle = 90.0;
      lastUpdate = 0;
    }

    void attachAndInitialize(int pin) {
      attach(pin);      
      write((int)currentAngle);
    }

    void angleLimits(int min, int max) {
      minAngle = min;
      maxAngle = max;
    }

    void servospeed(int speedVal) {
      unsigned long now = millis();
      if (now - lastUpdate < updateInterval) return;

      float delta = 0.0;
      int center = 514;

      if (abs(speedVal - center) > deadzone) {
        float range = (speedVal > center) ? (1023.0 - (center + deadzone)) : ((center - deadzone));
        float offset = (float)(speedVal - center) / range;
        delta = offset * maxSpeed; // Value between -maxSpeed to +maxSpeed
      }

      currentAngle += delta;
      currentAngle = constrain(currentAngle, minAngle, maxAngle);
      write((int)currentAngle);

      lastUpdate = now;
    }
};

int pinbut = 0;
int buttonstate = 0;
CustomServo baseservo;
CustomServo j1Servo;
CustomServo j2Servo;
Servo gripperservo;

ControlMode mode = None;

ControlMode readControlMode() {
  if (digitalRead(pinSpec0)) {
    if (digitalRead(pinSpec1)) {
      return None;    // 11
    } else {
      return Exo;    // 10
    }
  } else {
    if (digitalRead(pinSpec1)) {
      return MiniArm;     // 01
    } else {
      return Gamepad; // 00
    }
  }
}


void setup() {
  Serial.begin(9600);
  pinMode(pinSpec0, INPUT_PULLUP);
  pinMode(pinSpec1, INPUT_PULLUP);

  pinMode(pinbut, INPUT_PULLUP);
  baseservo.attachAndInitialize(3);
  j1Servo.attachAndInitialize(5);
  j2Servo.attachAndInitialize(6);
  gripperservo.attach(9, 625, 2500);

  j1Servo.angleLimits(55, 135);
  j2Servo.angleLimits(65, 130);
}

void loop() {
  ControlMode newMode = readControlMode();

  if (mode != newMode) {
    mode = newMode;
    Serial.print("New control method connected: ");
    switch (mode) {
      case Gamepad:
          Serial.println("Gamepad");
        break;
  
      case MiniArm:
          Serial.println("Mini Arm");
        break;
 
      case Exo:
          Serial.println("Arm Apparatus");
        break;
      
      case None:
          Serial.println("None");
        break;
    }

    delay(2000);
    return;
  }

  int val0 = analogRead(A0);
  int val1 = analogRead(A1);
  int val2 = analogRead(A2);

  switch (mode) {
    case None:
        Serial.print("No control method connected ");
        Serial.print(digitalRead(pinSpec0));
        Serial.print(" / ");
        Serial.print(digitalRead(pinSpec1));
        Serial.print(" / ");
        Serial.print(analogRead(A0));
        Serial.print(" / ");
        Serial.print(analogRead(A1));
        Serial.print(" / ");
        Serial.println(analogRead(A2));
        delay(500);
        return;

    case Gamepad:
        baseservo.servospeed(val2);
        j1Servo.servospeed(val1);
        j2Servo.servospeed(val0);
      break;

    case MiniArm:
        int potbase = constrain(
          map(val0, 210, 900, 180, 0),
          0, 180
        );
        baseservo.write(potbase);
        //int potj1 = val1;
        //potj1 = constrain(potj1, 775, 1023);
        //j1Servo.write(constrain(map(potj1, 775, 1023, 0, 180), 55, 135));

        int potj1 = constrain(
          map(val1, 750, 360, 172, 50),
          62, 180
        );
        j1Servo.write(potj1);

        int potj2 = constrain(
          map(val2, 495, 980, 140, 40),
          30, 150
        );
        j2Servo.write(potj2);
      break;

    case Exo:
        Serial.println("Arm Apparatus not yet implemented");
      break;
  }
  
  // Serial.println(ryValue);
  // Serial.println(lyValue);
  // Serial.println(buttonstate);
  // Serial.println(j1Servo.read());
  // Serial.println(j2Servo.read());

  Serial.print("base = ");
  Serial.print(val0);
  Serial.print("  j1 = ");
  Serial.print(val1);
  Serial.print("  j2 = ");
  Serial.print(val2);

  buttonstate = digitalRead(pinbut);
  if (buttonstate == HIGH) 
  {
    gripperservo.write(0);  
    Serial.println("  grip = OPEN");
  }
  else
  {
    gripperservo.write(180);
    Serial.println("  grip = CLOSED");
}
}
