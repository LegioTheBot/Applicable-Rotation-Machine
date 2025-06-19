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

String veri;
uint8_t servoNo = 0;
uint16_t gelenAci = 0.0;
uint16_t gelenDeger = 0;

void setup()
{
  Serial.begin(115200);

  pwm.begin();

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
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

void loop()
{
  if(pcDebugControl == true)
  {
    pcTerminalControl();
  }
  else
  {
    startPosition();
    while(1)
    {
      comm();
      servoControl();
    }
  }
}
