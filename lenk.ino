/*
  LENK — Arduino Mega 2560, TB6612FNG, MG996R, iki kanallı enkoder,
  6 x HC-SR04. Arduino IDE: "Arduino Mega or Mega 2560" + Servo kütüphanesi.

  ÖNERİLEN BAĞLANTI (Mega dijital pinleri)
  D2/D3: Enkoder A/B. Enkoderin besleme ve sinyal seviyesini doğrula.
  D5: PWMA. D30/D31: AIN1/AIN2. D32: STBY. AO1/AO2: DC motor.
  D6: MG996R sinyal. Servoyu akım kapasitesi uygun ayrı regülatörden besle.
  D7: normalde kapalı durdurma anahtarı -> GND. Açık hat hareketi engeller.
  HC-SR04 TRIG/ECHO:
    Ön       D22/D23    Arka      D24/D25
    Sağ ön   D26/D27    Sağ yan   D28/D29
    Sağ arka D33/D34    Sol yan   D35/D36
  Sağ ön ve sağ arka sensörler köşelere bakar, sağ yan boşluğu tarar.
  HC-SR04 VCC ve TB6612 VCC: 5 V mantık. VM: doğrulanmış motor beslemesi.
  Mega, sensörler, sürücü ve servo beslemesinin GND uçlarını ortakla.
  STBY'ı reset sırasında düşük tutmak için D32 ile GND arasına 10 kΩ bağla.
  Kullanılmayan B kanalının BIN1, BIN2 ve PWMB girişlerini düşük tut.
  Motor ve servo yüklerini Mega üzerinden besleme. 12 V etiketli motorun
  2S pil ile davranışını ve sürücünün akım sınırlarını tezgâhta doğrula.
  D7 yazılım tarafından izlenir. Deneme düzeneğinde motor beslemesini
  kesebileceğin bir anahtar erişilebilir olsun.

  SERİ MONİTÖR 115200: ? = ölçümler, P = parkı başlat, X = durdur.
  Sürüş kilitliyken C/R/L = servo merkez/sağ/sol (yalnız geçerli ayar).
  İlk açılışta motor hareketi kilitlidir. Önce tekerlekler havadayken pinleri,
  enkoder sayım yönünü ve servo hareket sınırlarını doğrula.
  CALIBRATION bölümünde ölçüm değerlerini gir, sonra DRIVE_ENABLED aç.
  Ölçüm için tekerleği elle ileri döndür ve ? ile enkoderi oku. İleri
  döndürünce sayım artmalı. 10 tam tekerlek turundaki değişimi 10'a böl.
  Dönüş yayı mesafeleri iki ayrı düşük hız denemesiyle kalibre edilir.
  Sağa paralel park içindir. İlk konum sağda ilk engelin yanı olmalı.
  Timeout veya eski veri ilgili kanalda hareketi durdurur. Açık alan ile
  yankısız engel ayırt edilemediği için kontrollü park düzeneğinde gerekli
  kanallarda ölçülebilir yansıtıcı yüzey kullan. Çıkış PARKED, kalibre
  edilmiş iki yayın tamamlandığını bildirir. Son konumu fiziksel ölç.

  Mimari referansları (algoritma fikirleri, kaynak kod burada özgündür):
  DIT112 Smartcar: https://github.com/DIT112-V19/group-08
  ESP32: https://github.com/444Nisco/self-parking-car-esp32
  Self-Parking-Car: https://github.com/vv258/Self-Parking-Car
*/

#include <Arduino.h>
#include <Servo.h>
#include <util/atomic.h>

enum Sensor : uint8_t { FRONT, REAR, RIGHT_FRONT, RIGHT_SIDE, RIGHT_REAR, LEFT_SIDE, SENSOR_COUNT };
enum State : uint8_t { IDLE, SEARCH, PULL_AHEAD, SET_RIGHT, ARC_RIGHT, SET_LEFT, ARC_LEFT, PARKED, ABORTED };

const uint8_t ENCODER_A = 2, ENCODER_B = 3;
const uint8_t MOTOR_PWM = 5, SERVO_PIN = 6, STOP_SWITCH = 7;
const uint8_t MOTOR_IN1 = 30, MOTOR_IN2 = 31, MOTOR_STBY = 32;
const uint8_t triggerPins[SENSOR_COUNT] = {22, 24, 26, 28, 33, 35};
const uint8_t echoPins[SENSOR_COUNT] = {23, 25, 27, 29, 34, 36};

// CALIBRATION — 0 bırakılan zorunlu alanlar P komutunu kilitler.
const bool DRIVE_ENABLED = false;
const bool MOTOR_REVERSED = false;  // İleri PWM aracı geri sürüyorsa değiştir.
const int8_t ENCODER_SIGN = 1;       // Elle ileri çevirince sayım azalıyorsa -1.
const float TICKS_PER_WHEEL_REV = 0; // A ve B'nin tüm kenarları sayılır.
const float WHEEL_CIRCUMFERENCE_CM = 29.85f; // 95 mm nominal çap; yuvarlanarak ölç.
const float VEHICLE_LENGTH_CM = 0;
const float GAP_MARGIN_CM = 40;       // Checkpoint 1 test düzeneğinin ek payı.
const float PULL_AHEAD_CM = 0;        // Boşluğun sonundan manevra başlangıcına.
const float RIGHT_ARC_CM = 0;         // Sağ direksiyonla geriye alınacak yol.
const float LEFT_ARC_CM = 0;          // Sol direksiyonla geriye alınacak yol.
const uint16_t SERVO_CENTER_US = 1500, SERVO_RIGHT_US = 0, SERVO_LEFT_US = 0;
const uint8_t SEARCH_PWM = 65, REVERSE_PWM = 60; // Başlangıç deneme değerleri.
const float SIDE_NEAR_CM = 26, SIDE_OPEN_CM = 45;
const float FRONT_STOP_CM = 20, REAR_STOP_CM = 20, SIDE_STOP_CM = 8;

const uint32_t ECHO_TIMEOUT_US = 12000;
const uint32_t PING_INTERVAL_US = 18000; // Kanallar sırayla; kanal başına ~108 ms.
const uint32_t SAMPLE_MAX_AGE_MS = 150;
const uint32_t STEERING_SETTLE_MS = 450, BRAKE_HOLD_MS = 220;
const uint32_t MAX_RUN_MS = 20000, ENCODER_STALL_MS = 850;
const float WRONG_DIRECTION_CM = 2;

struct Reading {
  float cm;
  uint32_t timeMs;
  bool valid;
};

Reading readings[SENSOR_COUNT] = {};
Servo steering;
volatile int32_t encoderTicks = 0;
volatile uint8_t encoderPrevious = 0;
State state = IDLE;
const char *const stateNames[] = {
  "IDLE", "SEARCH", "PULL_AHEAD", "SET_RIGHT", "ARC_RIGHT",
  "SET_LEFT", "ARC_LEFT", "PARKED", "ABORTED"
};
const char *stopReason = "-";
uint8_t nextSensor = 0;
uint32_t lastPingUs = 0, sideSampleId = 0, usedSideSampleId = 0;
uint32_t runSinceMs = 0, stateSinceMs = 0, motionSinceMs = 0, brakeSinceMs = 0;
int32_t stateOriginTicks = 0, lastProgressTicks = 0;
bool moving = false, braking = false;

bool nearObjectSeen = false, gapOpen = false;
uint8_t nearSamples = 0, farSamples = 0;
int32_t gapStartTicks = 0, edgeTicks = 0;

uint8_t readEncoderAB() {
  return (digitalRead(ENCODER_A) << 1) | digitalRead(ENCODER_B);
}

void onEncoderEdge() {
  static const int8_t step[16] = {
     0, 1,-1, 0, -1, 0, 0, 1, 1, 0, 0,-1, 0,-1, 1, 0
  };
  const uint8_t ab = readEncoderAB();
  encoderTicks += step[(encoderPrevious << 2) | ab];
  encoderPrevious = ab;
}

int32_t getTicks() {
  int32_t ticks;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { ticks = encoderTicks; }
  return ticks * ENCODER_SIGN;
}

float distanceCm(int32_t ticks) {
  return ticks * WHEEL_CIRCUMFERENCE_CM / TICKS_PER_WHEEL_REV;
}

void setMotorStandby() {
  digitalWrite(MOTOR_STBY, LOW);
  analogWrite(MOTOR_PWM, 0);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  braking = false;
}

void brakeMotor() {
  if (!moving) return;
  // TB6612FNG: STBY=HIGH, AIN1=AIN2=HIGH, PWMA=LOW -> short brake.
  analogWrite(MOTOR_PWM, 0);
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, HIGH);
  moving = false;
  braking = true;
  brakeSinceMs = millis();
}

void commandMotor(bool forward, uint8_t pwm) {
  if (!moving) {
    motionSinceMs = millis();
    lastProgressTicks = getTicks();
  }
  const bool in1 = forward != MOTOR_REVERSED;
  digitalWrite(MOTOR_IN1, in1);
  digitalWrite(MOTOR_IN2, !in1);
  digitalWrite(MOTOR_STBY, HIGH);
  analogWrite(MOTOR_PWM, pwm);
  moving = true;
  braking = false;
}

void enterState(State next) {
  state = next;
  stateSinceMs = millis();
  stateOriginTicks = getTicks();
}

bool activeRun() {
  return state >= SEARCH && state <= ARC_LEFT;
}

void abortRun(const char *reason) {
  brakeMotor();
  stopReason = reason;
  enterState(ABORTED);
}

void sampleOneSensor() {
  const uint32_t now = micros();
  if (now - lastPingUs < PING_INTERVAL_US) return;
  lastPingUs = now;
  const uint8_t i = nextSensor;
  nextSensor = (nextSensor + 1) % SENSOR_COUNT;

  digitalWrite(triggerPins[i], LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPins[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPins[i], LOW);
  const uint32_t duration = pulseIn(echoPins[i], HIGH, ECHO_TIMEOUT_US);
  // 0, timeout. Çok yakın bir yankıyı "açık alan" olarak kullanma.
  readings[i] = {duration * 0.01715f, millis(), duration != 0};
  if (i == RIGHT_SIDE) ++sideSampleId;
}

bool clearance(Sensor sensor, float minimumCm) {
  const Reading &reading = readings[sensor];
  return reading.valid && millis() - reading.timeMs <= SAMPLE_MAX_AGE_MS &&
         reading.cm > minimumCm;
}

bool forwardClear() {
  return clearance(FRONT, FRONT_STOP_CM) && clearance(RIGHT_FRONT, SIDE_STOP_CM) &&
         clearance(RIGHT_SIDE, SIDE_STOP_CM) && clearance(LEFT_SIDE, SIDE_STOP_CM);
}

bool reverseClear() {
  // Ön taraf da direksiyon kırarken dışarı doğru savrulur.
  return forwardClear() && clearance(REAR, REAR_STOP_CM) &&
         clearance(RIGHT_REAR, SIDE_STOP_CM);
}

bool validServoPulse(uint16_t pulseUs) {
  return pulseUs >= 1000 && pulseUs <= 2000;
}

void testSteering(uint16_t pulseUs) {
  if (DRIVE_ENABLED || digitalRead(STOP_SWITCH) != LOW || !validServoPulse(pulseUs)) return;
  steering.writeMicroseconds(pulseUs);
  if (!steering.attached()) steering.attach(SERVO_PIN);
}

bool calibrationReady() {
  return DRIVE_ENABLED && TICKS_PER_WHEEL_REV > 0 && VEHICLE_LENGTH_CM > 0 &&
         PULL_AHEAD_CM > 0 && RIGHT_ARC_CM > 0 && LEFT_ARC_CM > 0 &&
         validServoPulse(SERVO_CENTER_US) && validServoPulse(SERVO_RIGHT_US) &&
         validServoPulse(SERVO_LEFT_US) &&
         ((SERVO_RIGHT_US > SERVO_CENTER_US && SERVO_LEFT_US < SERVO_CENTER_US) ||
          (SERVO_RIGHT_US < SERVO_CENTER_US && SERVO_LEFT_US > SERVO_CENTER_US));
}

void beginParking() {
  if (activeRun() || braking) return;
  if (!calibrationReady()) {
    Serial.println(F("LOCKED: CALIBRATION"));
    return;
  }
  if (digitalRead(STOP_SWITCH) != LOW || !forwardClear()) {
    Serial.println(F("LOCKED: STOP switch / forward sonar"));
    return;
  }
  stopReason = "-";
  runSinceMs = millis();
  nearObjectSeen = false;
  gapOpen = false;
  nearSamples = farSamples = 0;
  usedSideSampleId = sideSampleId;
  steering.writeMicroseconds(SERVO_CENTER_US);
  enterState(SEARCH);
}

void printMeasurements() {
  // Araç hareket halindeyken seri çıktı ile kontrol çevrimini bloklama.
  if (activeRun() || braking) return;
  Serial.print(stateNames[state]);
  Serial.print(F(" reason="));
  Serial.print(stopReason);
  Serial.print(F(" ticks="));
  Serial.println(getTicks());
  Serial.print(F("cm [F R RF RS RR LS]:"));
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    Serial.print(' ');
    if (readings[i].valid && millis() - readings[i].timeMs <= SAMPLE_MAX_AGE_MS) {
      Serial.print(readings[i].cm, 1);
    } else {
      Serial.print('-');
    }
  }
  Serial.println();
}

bool readSerialCommands() {
  bool startRequested = false, stopRequested = false;
  uint16_t testPulse = 0;
  // Uno/Mega varsayılan seri alım kuyruğu boyutuyla sınırlı çalışma.
  for (uint8_t n = 0; n < 64 && Serial.available(); ++n) {
    const int ch = Serial.read();
    if (ch == 'X' || ch == 'x') stopRequested = true;
    if (ch == 'P' || ch == 'p') startRequested = true;
    if (ch == '?') printMeasurements();
    if (ch == 'C' || ch == 'c') testPulse = SERVO_CENTER_US;
    if (ch == 'R' || ch == 'r') testPulse = SERVO_RIGHT_US;
    if (ch == 'L' || ch == 'l') testPulse = SERVO_LEFT_US;
  }
  if (stopRequested) {
    abortRun("serial stop");
    return false;
  }
  if (testPulse) testSteering(testPulse);
  return startRequested;
}

bool foundGap(int32_t ticks) {
  if (sideSampleId == usedSideSampleId) return false;
  usedSideSampleId = sideSampleId;
  const float cm = readings[RIGHT_SIDE].cm;
  if (cm <= SIDE_NEAR_CM) {
    if (nearSamples == 0 && gapOpen) edgeTicks = ticks;
    if (nearSamples < 2) ++nearSamples;
    farSamples = 0;
  } else if (cm >= SIDE_OPEN_CM) {
    if (farSamples == 0 && !gapOpen) edgeTicks = ticks;
    if (farSamples < 2) ++farSamples;
    nearSamples = 0;
  } else {
    nearSamples = farSamples = 0;
  }

  if (nearSamples >= 2) {
    nearObjectSeen = true;
    if (gapOpen) {
      gapOpen = false;
      return distanceCm(edgeTicks - gapStartTicks) >= VEHICLE_LENGTH_CM + GAP_MARGIN_CM;
    }
  }
  if (nearObjectSeen && !gapOpen && farSamples >= 2) {
    gapStartTicks = edgeTicks;
    gapOpen = true;
  }
  return false;
}

bool checkProgress(int32_t ticks, bool forward) {
  const int32_t delta = forward ? ticks - lastProgressTicks : lastProgressTicks - ticks;
  if (delta > 0) {
    lastProgressTicks = ticks;
    motionSinceMs = millis();
  } else if (distanceCm(delta) < -WRONG_DIRECTION_CM) {
    abortRun("encoder direction");
    return false;
  } else if (millis() - motionSinceMs >= ENCODER_STALL_MS) {
    abortRun("encoder no progress");
    return false;
  }
  return true;
}

void updateParking() {
  if (!activeRun()) return;

  const uint32_t now = millis();
  const bool forward = state == SEARCH || state == PULL_AHEAD;
  if (now - runSinceMs >= MAX_RUN_MS) {
    abortRun("run timeout");
    return;
  }
  if (forward ? !forwardClear() : !reverseClear()) {
    abortRun("clearance / sonar invalid");
    return;
  }

  const int32_t ticks = getTicks();
  if (moving && !checkProgress(ticks, forward)) return;
  const float traveled = distanceCm(ticks - stateOriginTicks);

  switch (state) {
    case SEARCH:
      if (foundGap(ticks)) enterState(PULL_AHEAD);
      commandMotor(true, SEARCH_PWM);
      break;
    case PULL_AHEAD:
      if (traveled >= PULL_AHEAD_CM) {
        brakeMotor();
        steering.writeMicroseconds(SERVO_RIGHT_US);
        enterState(SET_RIGHT);
      } else {
        commandMotor(true, SEARCH_PWM);
      }
      break;
    case SET_RIGHT:
    case SET_LEFT:
      if (now - stateSinceMs >= STEERING_SETTLE_MS) {
        enterState(state == SET_RIGHT ? ARC_RIGHT : ARC_LEFT);
      }
      break;
    case ARC_RIGHT:
      if (-traveled >= RIGHT_ARC_CM) {
        brakeMotor();
        steering.writeMicroseconds(SERVO_LEFT_US);
        enterState(SET_LEFT);
      } else {
        commandMotor(false, REVERSE_PWM);
      }
      break;
    case ARC_LEFT:
      if (-traveled >= LEFT_ARC_CM) {
        brakeMotor();
        steering.writeMicroseconds(SERVO_CENTER_US);
        enterState(PARKED);
      } else {
        commandMotor(false, REVERSE_PWM);
      }
      break;
    default:
      break;
  }
}

void setup() {
  digitalWrite(MOTOR_STBY, LOW);
  pinMode(MOTOR_STBY, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  setMotorStandby();
  pinMode(STOP_SWITCH, INPUT_PULLUP);

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    pinMode(triggerPins[i], OUTPUT);
    digitalWrite(triggerPins[i], LOW);
    pinMode(echoPins[i], INPUT);
  }
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  encoderPrevious = readEncoderAB();
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), onEncoderEdge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), onEncoderEdge, CHANGE);

  // Kalibrasyon modunda servo da hareketsiz kalır.
  if (calibrationReady()) {
    steering.writeMicroseconds(SERVO_CENTER_US);
    steering.attach(SERVO_PIN);
  }
  Serial.begin(115200);
  Serial.println(F("LENK: ? = readings, P = park, X = stop"));
}

void loop() {
  const bool start = readSerialCommands();
  if (activeRun() && digitalRead(STOP_SWITCH) != LOW) abortRun("stop switch");
  if (braking && millis() - brakeSinceMs >= BRAKE_HOLD_MS) setMotorStandby();

  sampleOneSensor();

  if (activeRun() && digitalRead(STOP_SWITCH) != LOW) abortRun("stop switch");
  if (start) beginParking();
  updateParking();
}
