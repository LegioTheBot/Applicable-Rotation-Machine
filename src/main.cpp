#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

bool pcDebugControl = false; // false olursa sistem normal algoritmada çalışır. True olursa serial terminal üzerinden Servo n0 boşluk açı şeklinde kontrol edilebilir. ( 0 125)

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define NUM_SERVOS 4
#define OMUZ   0
#define DIRSEK 1
#define BILEK  2
#define EL     3

#define PIN_SPEC0 7
#define PIN_SPEC1 6
#define PIN_BTN 5

enum ControlMode : uint8_t {
  Gamepad,
  MiniArm,
  ArmApparatus,
  None
};

// Servo limitleri
const uint16_t servoMin[NUM_SERVOS] = {100, 210, 215, 110};
const uint16_t servoMax[NUM_SERVOS] = {480, 400, 390, 470};

// Mevcut ve hedef pozisyonlar
uint16_t curPos[NUM_SERVOS]    = {290, 400, 290, 110};
uint16_t targetPos[NUM_SERVOS] = {290, 400, 290, 110};

// Zamanlayıcılar
unsigned long lastStepTime[NUM_SERVOS] = {0, 0, 0, 0};
// buradan servoların hızı ayarlanıyor.
const uint16_t stepDelay = 20;   // Her adım arası gecikme (ms)
const uint8_t stepSize   = 3;    // Adım boyutu (pulse)

unsigned long lastMoveTime = 0;
const uint32_t moveInterval = 1000;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

uint8_t servonum = 0;
unsigned long msControlLock = 0;
const unsigned long msControlLockDuration = 500;

int idx1 = 0;
int idx2 = 0;
int idx3 = 0;

// sisteme elekrik verdikten sonra öncelikle startPositon alt programı calışıp.
//sonrasında aşagıdaki değerlerde servolar beklemeye girecek
// pyton dan veri gelirse hareket edecek.
int baseDeg = 90;
int j1Deg   = 90;
int j2Deg   = 90;
int gripDeg = 90;

const int joystickDeadzone = 30;        // Joystick deadzone around center (514)
unsigned long controlInterval = 0;
int a0Val = 0;
int a1Val = 0;
int a2Val = 0;

const uint8_t debounceTicks = 6;
uint8_t btnVal = 0;

String veri;
uint8_t servoNo = 0;
uint16_t gelenAci = 0.0;
uint16_t gelenDeger = 0;
ControlMode controlMode = None;
ControlMode nextControlMode = None;

ControlMode readControlMode() {
  if (digitalRead(PIN_SPEC0)) {
    if (digitalRead(PIN_SPEC1)) {
      return None;    // 11
    } else {
      return ArmApparatus;    // 10
    }
  } else {
    if (digitalRead(PIN_SPEC1)) {
      return MiniArm;     // 01
    } else {
      return Gamepad; // 00
    }
  }
}

// Hareket için zaman geldi mi kontrolü
bool shouldStep(uint8_t ch)
{
  return (millis() - lastStepTime[ch] >= stepDelay);
}

// Servo bir adım yaklaştır
void updateServo(uint8_t ch) {
  if (curPos[ch] == targetPos[ch]) return;
  if (!shouldStep(ch)) return;

  int dir = (targetPos[ch] > curPos[ch]) ? 1 : -1;
  curPos[ch] += dir * stepSize;

  // Sınır aşımını önle
  if ((dir > 0 && curPos[ch] > targetPos[ch]) ||
      (dir < 0 && curPos[ch] < targetPos[ch])) {
    curPos[ch] = targetPos[ch];
  }

  pwm.setPWM(ch, 0, curPos[ch]);
  lastStepTime[ch] = millis();
}

void startPosition()
{
  for (int i = 0; i < NUM_SERVOS; i++)
  {
    pwm.setPWM(i, 0, curPos[i]);
  }
  delay(2000); // başlangıc posizyonuna gelmesini için bekle
}

uint16_t mapDegreeToPulse(int degree)
{
  degree = constrain(degree, 0, 180);
  return (uint16_t)((400L * degree) / 180 + 95);
}

void pcTerminalControl()
{
  if (Serial.available())
  {
    veri = Serial.readStringUntil('\n');      // Satırı oku
    veri.trim();                              // Gereksiz boşlukları temizle

    int boslukIndex = veri.indexOf(' ');
    if (boslukIndex == -1) {
      Serial.println("Hatalı giriş! Format: <servo_no> <aci>");
      return;
    }

    String sNo  = veri.substring(0, boslukIndex);
    String sAci = veri.substring(boslukIndex + 1);

    servoNo  = sNo.toInt();
    gelenAci = sAci.toInt();

    if (servoNo > 15 || gelenAci < 0 || gelenAci > 180) {
      Serial.println("Geçersiz servo numarası veya açı aralığı!");
      return;
    }

    gelenDeger = mapDegreeToPulse(gelenAci);
    pwm.setPWM(servoNo, 0, gelenDeger);

    Serial.print("Servo ");
    Serial.print(servoNo);
    Serial.print(" → ");
    Serial.print(gelenAci);
    Serial.print(" derece = ");
    Serial.print(gelenDeger);
    Serial.println(" pulse");
  }
}

void comm()
{
  if (Serial.available())
  {
    String input = Serial.readStringUntil('\n');
    if (input.startsWith("#"))
    {
      input.remove(0, 1);
      idx1 = input.indexOf(',');
      idx2 = input.indexOf(',', idx1 + 1);
      idx3 = input.indexOf(',', idx2 + 1);

      baseDeg = input.substring(0, idx1).toInt();          // omuz
      j1Deg   = input.substring(idx1 + 1, idx2).toInt();   // dirsek
      j2Deg   = input.substring(idx2 + 1, idx3).toInt();   // bilek
      gripDeg = input.substring(idx3 + 1).toInt();         // el
    }
  }
}

void servoControl()
{
  targetPos[OMUZ]   = mapDegreeToPulse(baseDeg);
  targetPos[DIRSEK] = mapDegreeToPulse(j1Deg);
  targetPos[BILEK]  = mapDegreeToPulse(j2Deg);
  targetPos[EL]     = mapDegreeToPulse(gripDeg);

  // Tüm servoları güncelle
  for (int i = 0; i < NUM_SERVOS; i++)
  {
    updateServo(i);
  }
}

bool controlModeChanged()
{
  nextControlMode = readControlMode();

  if (nextControlMode == controlMode)
  {
    return false;
  }
  else if (msControlLock == 0)
  {
    msControlLock = millis() + msControlLockDuration;
  }
  else if (millis() > msControlLock)
  {
    controlMode = nextControlMode;
    msControlLock = 0;

    Serial.print("New control method connected: ");
    switch (controlMode) {
      case Gamepad:
          Serial.println("Gamepad");
        break;

      case MiniArm:
          Serial.println("Mini Arm");
        break;

      case ArmApparatus:
          Serial.println("Arm Apparatus");
        break;

      case None:
          Serial.println("None");
        break;
    }
    return true;
  }

  return false;
}

// Additive rotation
void servoJoystick(int* currentAngle, int speedVal) {
  float delta = 0.0f;
  int center = 514;

  if (abs(speedVal - center) > joystickDeadzone) {
    float range = (speedVal > center) ? (1023.0 - (center + joystickDeadzone)) : ((center - joystickDeadzone));
    float offset = (float)(speedVal - center) / range;
    delta = offset * 1.5f; // Value between -maxSpeed to +maxSpeed
  }

  *currentAngle += delta;
}

void useControlMethod()
{
  if (millis() < controlInterval){
    return;
  }
  controlInterval = millis() + 20;

  a0Val = analogRead(A0);
  a1Val = analogRead(A1);
  a2Val = analogRead(A2);

  Serial.print(digitalRead(PIN_SPEC0));
  Serial.print(" / ");
  Serial.print(digitalRead(PIN_SPEC1));
  Serial.print(" / ");
  Serial.print(digitalRead(PIN_BTN));
  Serial.print(" / ");
  Serial.print(a0Val);
  Serial.print(" / ");
  Serial.print(a1Val);
  Serial.print(" / ");
  Serial.println(a2Val);

  switch (controlMode)
  {
    case None:
        Serial.print("No control method connected ");
        return;

    case Gamepad:
        servoJoystick(&baseDeg, a0Val);
        servoJoystick(&j1Deg, a1Val);
        servoJoystick(&j2Deg, a2Val);
      break;

    case MiniArm:
        baseDeg = constrain(
          map(a0Val, 210, 900, 0, 180),
          0, 180
        );
        j1Deg = constrain(
          map(a1Val, 256, 560, 50, 172),
          62, 180
        );
        j2Deg = constrain(
          map(a2Val, 40, 530, 140, 40),
          30, 150
        );
      break;

    case ArmApparatus:
        baseDeg = constrain(
          map(a1Val, 210, 600, 0, 180),
          0, 180
        );
        j1Deg = constrain(
          map(a0Val, 390, 850, 155, 65),
          30, 150
        );
        j2Deg = constrain(
          map(a2Val, 570, 920, 50, 172),
          62, 180
        );
      break;
  }

  // Gripper happens after everything, we return early on None, so automatic and manual control don't conflict
  btnVal = constrain(btnVal + (digitalRead(PIN_BTN) ? 1 : -1), 0, debounceTicks);
  if (btnVal == debounceTicks && gripDeg != 90) {
    gripDeg = 90;
  }
  else if (btnVal == 0 && gripDeg != 170) {
    gripDeg = 170;
  }
}


void setup()
{
  Serial.begin(115200);

  pwm.begin();

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  pinMode(PIN_SPEC0, INPUT_PULLUP);
  pinMode(PIN_SPEC1, INPUT_PULLUP);
  pinMode(PIN_BTN, INPUT_PULLUP);
  delay(10);

  startPosition();
}

void loop()
{
  if(pcDebugControl == true)
  {
    pcTerminalControl();
  }
  else if (controlModeChanged() == false)
  {
    //useControlMethod();
    comm();
    servoControl();
  }
}
