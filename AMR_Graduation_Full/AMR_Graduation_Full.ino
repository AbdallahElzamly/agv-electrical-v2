// ============================================================================
//  مشروع التخرج: روبوت AMR ذكي لتتبع الخط، تفادي العوائق، ورفع الحمولة (كيس شيبسي)
//  AMR Graduation Project Firmware — Rev C (2026)
//  البوردة: ESP32 DevKit V1 (30-Pin) — ESP32 Arduino Core 3.x ("ESP32 Dev Module")
// ----------------------------------------------------------------------------
//  المواصفات المعمارية:
//  - الدفع: عجلتين خلفيتين (Differential Drive) + عجلة حرة أمامية (Caster Wheel)
//  - الدرايفر: متوافق مع L298N ومع البديل الأرخص والأكفأ TB6612FNG بنفس التوصيل
//  - تتبع الخط: 3 حساسات IR TCRT5000 (شمال GPIO34، وسط GPIO35، يمين GPIO19)
//  - الأمان وتفادي الاصطدام: 2 حساس ألتراسونيك HC-SR04
//      * يمين (TRIG_R = 23, ECHO_R = 22 عبر مقسم 1k/2k)
//      * شمال (TRIG_L = 21, ECHO_L = 17 عبر مقسم 1k/2k)
//  - آلية الرفع: سيرفو 180° (MG996R أو MG90S Metal) على GPIO18 (تغذية منفصلة BK2)
//  - مراقبة البطارية: مقسم 100k/33k على GPIO39 (VN) لحماية البطارية من التفريغ العميق
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include "esp_system.h"

// ======================== 1. تعريف أطراف التوصيل (Pinout) ========================
// الموتور الشمال (M1)
const int ENA = 25;   // PWMA (مع مقاومة 10k للأرضي)
const int IN1 = 26;   // AIN1
const int IN2 = 27;   // AIN2

// الموتور اليمين (M2)
const int ENB = 33;   // PWMB (مع مقاومة 10k للأرضي لمنع نبضة الإقلاع)
const int IN3 = 32;   // BIN1
const int IN4 = 14;   // BIN2

// سيرفو رفع وإنزال كيس الشيبسي
const int SERVO_PIN = 18; // عبر مقاومة 1k توالي + 10k للأرضي

// حساسات الألتراسونيك للأمان (يمين وشمال)
const int TRIG_R = 23;
const int ECHO_R = 22;    // عبر مقسم جهد 1kΩ / 2kΩ (هام جداً!)
const int TRIG_L = 21;
const int ECHO_L = 17;    // عبر مقسم جهد 1kΩ / 2kΩ (هام جداً!)

// حساسات تتبع الخط الثلاثة (TCRT5000) - تغذية 3.3V من الـ ESP32
const int IR_L = 34;      // حساس شمال
const int IR_C = 35;      // حساس وسط
const int IR_R = 19;      // حساس يمين

// مراقبة البطارية وأزرار البوردة
const int VBAT_PIN = 39;  // طرف VN (مقسم 100k / 33k)
const int BOOT_BTN = 0;   // زرار BOOT المدمج
const int LED_PIN  = 2;   // اللمبة الزرقاء المدمجة

// ======================== 2. معاملات المعايرة (تتعدل من اختبار البنش) ========================
float TRIM_L      = 1.00f;  // معامل تصحيح سرعة الموتور الشمال
float TRIM_R      = 1.00f;  // معامل تصحيح سرعة الموتور اليمين
int   SERVO_US_0  = 500;    // عرض النبضة لزاوية 0° (ذراع الرفع تحت / استلام أو وضع الشيبسي)
int   SERVO_US_90 = 1500;   // عرض النبضة لزاوية 90° (ذراع الرفع فوق / شايل كيس الشيبسي)
float VBAT_CAL    = 1.000f; // معامل معايرة قراءة البطارية
const int IR_BLACK = HIGH;  // قراءة حساس الـ IR فوق الشريط الأسود (HIGH أو LOW حسب الموديول)

// سرعات الحركة (نسبة مئوية من 0.0 إلى 1.0)
// ملاحظة: مع L298N ابدأ بـ 0.48، ومع TB6612FNG يمكنك البدء بـ 0.38 لأن الفقد أقل
float BASE_SPEED  = 0.48f;  // السرعة الأساسية على الخط المستقيم
float TURN_FAST   = 0.55f;  // سرعة العجلة الخارجية في المنحنى
float TURN_SLOW   = 0.15f;  // سرعة العجلة الداخلية في المنحنى
float SEARCH_SPD  = 0.42f;  // سرعة البحث عن الخط لو ضاع لحظياً

// مسافات الأمان للألتراسونيك (بالسنتيمتر)
const float SAFE_STOP_CM = 20.0f; // يقف فوراً لو عائق على يمينه أو شماله أقل من 20 سم
const float SAFE_GO_CM   = 25.0f; // يرجع يكمل مشي لما العائق يبعد عن 25 سم (Hysteresis)

// حدود البطارية (2S Li-ion: 6.0V - 8.4V)
const float V_NOMINAL = 7.4f;
const float V_WARN    = 7.0f;
const float V_CUT     = 6.6f;
const float DIV_RATIO = (100.0f + 33.0f) / 33.0f; // 4.0303

// إعدادات الـ PWM
const int PWM_FREQ   = 1000;
const int PWM_BITS   = 10;   // 0..1023
const int SERVO_FREQ = 50;
const int SERVO_BITS = 14;   // 0..16383

// ======================== 3. حالات الروبوت (State Machine) ========================
enum RobotState {
  STATE_IDLE,            // واقف مستني ضغطة زرار BOOT للانطلاق (أو أوامر Serial)
  STATE_PICKUP_CHIPSY,   // في محطة البداية: بيرفع كيس الشيبسي لفوق (0° -> 90°)
  STATE_FOLLOW_LINE,     // ماشي بيتتبع الخط الأسود وبيراقب الألتراسونيك اليمين والشمال
  STATE_OBSTACLE_WAIT,   // واقف مؤقتاً عشان فيه عائق يمين أو شمال لحد ما العائق يتشال
  STATE_STATION_DROP,    // وصل محطة التسليم (الـ 3 حساسات شافوا خط عرضي أسود): بينزل كيس الشيبسي
  STATE_LOW_BATTERY      // البطارية أقل من 6.6V: إيقاف آمن للمواتير لحماية الخلايا
};

RobotState state = STATE_IDLE;
RobotState prevStateBeforeObstacle = STATE_FOLLOW_LINE;
bool servoAttached = false;
bool carryingChipsy = false;
int  lastLineDir = 0; // -1 آخر مرة الخط كان شمال، +1 آخر مرة كان يمين، 0 في النص
unsigned long lastUltraMs = 0;
unsigned long stationFilterMs = 0;
float distRightCm = 999.0f;
float distLeftCm  = 999.0f;

// ======================== 4. دوال الهاردوير والتحكم ========================

// قراءة جهد البطارية بمتوسط 16 قراءة
float readVbat() {
  uint32_t mv = 0;
  for (int i = 0; i < 16; i++) {
    mv += analogReadMilliVolts(VBAT_PIN);
  }
  return (mv / 16.0f) / 1000.0f * DIV_RATIO * VBAT_CAL;
}

// إيقاف الموتورين فوراً
void motorsStop() {
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// تحريك موتور واحد مع تعويض هبوط جهد البطارية أوتوماتيكياً
void motorDrive(bool isLeft, float pct, int dir) {
  if (pct <= 0.01f || dir == 0) {
    int en = isLeft ? ENA : ENB;
    int a  = isLeft ? IN1 : IN3;
    int b  = isLeft ? IN2 : IN4;
    ledcWrite(en, 0);
    digitalWrite(a, LOW);
    digitalWrite(b, LOW);
    return;
  }
  float vb = readVbat();
  float trim = isLeft ? TRIM_L : TRIM_R;
  float compensated = pct * trim * (V_NOMINAL / max(vb, V_CUT));
  if (compensated > 1.0f) compensated = 1.0f;

  int duty = (int)(compensated * ((1 << PWM_BITS) - 1));
  int a  = isLeft ? IN1 : IN3;
  int b  = isLeft ? IN2 : IN4;
  int en = isLeft ? ENA : ENB;

  digitalWrite(a, dir > 0 ? HIGH : LOW);
  digitalWrite(b, dir > 0 ? LOW  : HIGH);
  ledcWrite(en, duty);
}

// تحريك العجلتين معاً
void driveLR(float leftPct, int leftDir, float rightPct, int rightDir) {
  motorDrive(true,  leftPct,  leftDir);
  motorDrive(false, rightPct, rightDir);
}

// التحكم في السيرفو بالميكروثانية والزاوية (مع حركة ناعمة Smooth Lift عشان كيس الشيبسي ميقعش والكهربا متتهزش)
void servoWriteUs(int us) {
  if (!servoAttached) {
    ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_BITS);
    servoAttached = true;
  }
  uint32_t duty = (uint32_t)us * ((1 << SERVO_BITS) - 1) / 20000UL;
  ledcWrite(SERVO_PIN, duty);
}

int angleToUs(float deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  return SERVO_US_0 + (int)((SERVO_US_90 - SERVO_US_0) * (deg / 90.0f));
}

// تحريك السيرفو تدريجياً (Soft Start/Stop) لتقليل تيار البدء وحماية الكيس
void servoMoveSmooth(float fromDeg, float toDeg, int stepDelayMs = 18) {
  // قاعدة ذهبية من E-02: ممنوع السيرفو يتحرك والمواتير شغالة!
  motorsStop();
  int steps = 30;
  for (int i = 0; i <= steps; i++) {
    float cur = fromDeg + (toDeg - fromDeg) * ((float)i / steps);
    servoWriteUs(angleToUs(cur));
    delay(stepDelayMs);
  }
}

// قراءة حساس ألتراسونيك واحد (بالمتر/سنتيمتر) مع Timeout 20ms لمنع تعليق الكود
float readUltrasonicOnce(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH, 20000UL); // 20ms ~ 3.4m max
  if (duration == 0) return 999.0f; // مفيش عائق قريب
  return duration * 0.0343f / 2.0f;
}

// وسيط 3 قراءات لمنع القراءات الكاذبة
float readUltrasonicMedian(int trigPin, int echoPin) {
  float a[3];
  for (int i = 0; i < 3; i++) {
    a[i] = readUltrasonicOnce(trigPin, echoPin);
    delay(15);
  }
  for (int i = 0; i < 2; i++)
    for (int j = i + 1; j < 3; j++)
      if (a[j] < a[i]) { float t = a[i]; a[i] = a[j]; a[j] = t; }
  return a[1];
}

// تحديث قراءات الألتراسونيك اليمين والشمال بالتناوب كل 60 ملي ثانية أثناء المشي
void updateSafetyUltrasonics() {
  static bool readRightNext = true;
  if (millis() - lastUltraMs < 60) return;
  lastUltraMs = millis();

  if (readRightNext) {
    distRightCm = readUltrasonicOnce(TRIG_R, ECHO_R);
  } else {
    distLeftCm  = readUltrasonicOnce(TRIG_L, ECHO_L);
  }
  readRightNext = !readRightNext;
}

void blinkLED(int count, int onMs = 150, int offMs = 150) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(onMs);
    digitalWrite(LED_PIN, LOW);
    delay(offMs);
  }
}

const char* resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:  return "POWERON (تشغيل طبيعي)";
    case ESP_RST_BROWNOUT: return "BROWNOUT (هبوط جهد! راجع BK1/BK2)";
    case ESP_RST_SW:       return "SW (رفع كود)";
    default:               return "OTHER";
  }
}

// ======================== 5. أوامر اختبار البنش عبر الـ Serial ========================
void printHelp() {
  Serial.println(F("\n================ AMR Graduation Robot (Rev C) ================"));
  Serial.println(F("Commands via Serial Monitor (115200 baud):"));
  Serial.println(F("  i : Test 3 IR Line Sensors (L=34, C=35, R=19)"));
  Serial.println(F("  u : Test Right & Left Ultrasonic Sensors (R=23/22, L=21/17)"));
  Serial.println(F("  b : Test Battery Voltage (GPIO39 / VN)"));
  Serial.println(F("  s : Test Servo Lift (0 deg <-> 90 deg smooth)"));
  Serial.println(F("  g : Start / Stop Autonomous AMR Mission (Line + Lift + Safety)"));
  Serial.println(F("  h : Print this help menu"));
  Serial.println(F("Or press onboard BOOT button (GPIO0) to start autonomous run!"));
  Serial.println(F("=============================================================="));
}

void runSerialBenchCommand(char cmd) {
  if (cmd == 'i') {
    Serial.println(F("\n[IR Line Sensors Test - 15 readings]"));
    for (int k = 0; k < 15; k++) {
      bool bL = (digitalRead(IR_L) == IR_BLACK);
      bool bC = (digitalRead(IR_C) == IR_BLACK);
      bool bR = (digitalRead(IR_R) == IR_BLACK);
      Serial.printf("L=%s | C=%s | R=%s %s\n",
        bL ? "BLACK" : "white",
        bC ? "BLACK" : "white",
        bR ? "BLACK" : "white",
        (bL && bC && bR) ? " << [STATION Cross Line!]" : "");
      delay(250);
    }
  } else if (cmd == 'u') {
    Serial.println(F("\n[Dual Ultrasonic Safety Test - 10 readings]"));
    for (int k = 0; k < 10; k++) {
      float dR = readUltrasonicMedian(TRIG_R, ECHO_R);
      float dL = readUltrasonicMedian(TRIG_L, ECHO_L);
      Serial.printf("Right US: %6.1f cm | Left US: %6.1f cm %s\n",
        dR, dL, (dR <= SAFE_STOP_CM || dL <= SAFE_STOP_CM) ? " << [OBSTACLE STOP!]" : "[CLEAR]");
      delay(200);
    }
  } else if (cmd == 'b') {
    Serial.println(F("\n[Battery Voltage Test]"));
    for (int k = 0; k < 5; k++) {
      float vb = readVbat();
      Serial.printf("VBAT = %.2f V %s\n", vb,
        vb < V_CUT ? "<< CRITICAL (<6.6V) CHARGE NOW!" : (vb < V_WARN ? "<< LOW (<7.0V)" : "<< OK"));
      delay(300);
    }
  } else if (cmd == 's') {
    Serial.println(F("\n[Servo Chipsy Lift Test: 0 deg -> 90 deg -> 0 deg]"));
    servoMoveSmooth(0, 90, 20);
    delay(1500);
    servoMoveSmooth(90, 0, 20);
    Serial.println(F("Servo Test Done."));
  } else if (cmd == 'g') {
    if (state == STATE_IDLE) {
      Serial.println(F(">>> Starting Autonomous AMR Mission!"));
      state = STATE_PICKUP_CHIPSY;
    } else {
      motorsStop();
      state = STATE_IDLE;
      Serial.println(F(">>> Mission Stopped -> IDLE."));
    }
  } else if (cmd == 'h') {
    printHelp();
  }
}

// ======================== 6. الإعداد الأولي (setup) ========================
void setup() {
  // قفل الـ WiFi والبلوتوث لتوفير حوالي 150mA وتثبيت جهد الـ 5V
  WiFi.mode(WIFI_OFF);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BOOT_BTN, INPUT_PULLUP);

  // أطراف المواتير
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  ledcAttach(ENA, PWM_FREQ, PWM_BITS);
  ledcAttach(ENB, PWM_FREQ, PWM_BITS);
  motorsStop();

  // أطراف حساسات الألتراسونيك (يمين وشمال)
  pinMode(TRIG_R, OUTPUT); digitalWrite(TRIG_R, LOW);
  pinMode(ECHO_R, INPUT);
  pinMode(TRIG_L, OUTPUT); digitalWrite(TRIG_L, LOW);
  pinMode(ECHO_L, INPUT);

  // أطراف حساسات تتبع الخط
  pinMode(IR_L, INPUT);
  pinMode(IR_C, INPUT);
  pinMode(IR_R, INPUT);

  // إعداد الـ ADC لقراءة البطارية
  analogSetPinAttenuation(VBAT_PIN, ADC_11db);

  Serial.begin(115200);
  delay(300);
  Serial.printf("\nBooting AMR... Reset Reason: %s\n", resetReasonStr(esp_reset_reason()));

  // وضع ذراع السيرفو في الوضع السفلي (0°) عند التشغيل
  servoWriteUs(angleToUs(0));
  carryingChipsy = false;

  // إشارة الإقلاع السليم (لمبة زرقاء لمدة ثانيتين)
  digitalWrite(LED_PIN, HIGH);
  delay(2000);
  digitalWrite(LED_PIN, LOW);

  printHelp();
}

// ======================== 7. الحلقة الرئيسية (loop) ========================
void loop() {
  // 1) استقبال أوامر الاختبار من اللابتوب (Serial Monitor)
  if (Serial.available()) {
    char c = Serial.read();
    runSerialBenchCommand(c);
  }

  // 2) زرار BOOT على البوردة: يبدأ المهمة أو يوقف الروبوت
  if (digitalRead(BOOT_BTN) == LOW) {
    delay(40);
    while (digitalRead(BOOT_BTN) == LOW) delay(10);
    if (state == STATE_IDLE) {
      // عد تنازلي 3 رمشات قبل الانطلاق
      blinkLED(3, 300, 300);
      state = STATE_PICKUP_CHIPSY;
    } else {
      motorsStop();
      state = STATE_IDLE;
      blinkLED(2, 100, 100);
    }
  }

  // 3) فحص حماية البطارية أثناء التشغيل
  static unsigned long lastBatCheck = 0;
  if (state != STATE_IDLE && state != STATE_LOW_BATTERY && (millis() - lastBatCheck > 1000)) {
    lastBatCheck = millis();
    float vb = readVbat();
    if (vb > 2.5f && vb < V_CUT) { // لو فوق 2.5V يعني البطارية متوصلة وفعلاً ضعيفة
      motorsStop();
      state = STATE_LOW_BATTERY;
      Serial.printf("[ALERT] Low Battery (%.2fV < %.2fV)! Stopping motors.\n", vb, V_CUT);
    }
  }

  // 4) ماكينة الحالات (State Machine) للروبوت
  switch (state) {
    case STATE_IDLE:
      motorsStop();
      break;

    case STATE_LOW_BATTERY:
      motorsStop();
      // رعشة سريعة متكررة للمبة الزرقاء للتنبيه إن البطارية عايزة تتشحن
      blinkLED(1, 60, 60);
      break;

    case STATE_PICKUP_CHIPSY:
      // الروبوت واقف في بداية المسار: يرفع كيس الشيبسي من 0° لـ 90° بنعومة قبل ما يتحرك
      motorsStop();
      Serial.println(F("[STATE] Picking up Chipsy bag (0 -> 90 deg)..."));
      servoMoveSmooth(0, 90, 20);
      carryingChipsy = true;
      delay(500);
      // دفعة خفيفة للأمام لمدة 250 ملي ثانية عشان يتخطى خط محطة البداية العرضي
      driveLR(BASE_SPEED, +1, BASE_SPEED, +1);
      delay(250);
      stationFilterMs = millis();
      Serial.println(F("[STATE] Following line + Active Ultrasonic Safety (Right & Left)..."));
      state = STATE_FOLLOW_LINE;
      break;

    case STATE_FOLLOW_LINE: {
      // أ) فحص حساسات الألتراسونيك (يمين وشمال) للأمان
      updateSafetyUltrasonics();
      if (distRightCm <= SAFE_STOP_CM || distLeftCm <= SAFE_STOP_CM) {
        motorsStop();
        digitalWrite(LED_PIN, HIGH); // نور مستمر يعني واقف بسبب عائق
        Serial.printf("[SAFETY STOP] Obstacle detected! Right=%.1f cm, Left=%.1f cm\n", distRightCm, distLeftCm);
        prevStateBeforeObstacle = STATE_FOLLOW_LINE;
        state = STATE_OBSTACLE_WAIT;
        break;
      }

      // ب) قراءة حساسات تتبع الخط الثلاثة
      bool bL = (digitalRead(IR_L) == IR_BLACK);
      bool bC = (digitalRead(IR_C) == IR_BLACK);
      bool bR = (digitalRead(IR_R) == IR_BLACK);

      // ج) هل وصلنا علامة المحطة (الـ 3 حساسات شايفين أسود مع بعض بعد مرور ثانية على الأقل من الانطلاق)؟
      if (bL && bC && bR && (millis() - stationFilterMs > 1200)) {
        // تأكيد القراءة بعد 25 ملي ثانية لمنع الشوشرة
        delay(25);
        if (digitalRead(IR_L) == IR_BLACK && digitalRead(IR_C) == IR_BLACK && digitalRead(IR_R) == IR_BLACK) {
          motorsStop();
          Serial.println(F("[STATION] Reached destination station mark (111)!"));
          state = STATE_STATION_DROP;
          break;
        }
      }

      // د) خوارزمية تتبع الخط التفاضلية (Differential Steering)
      if (!bL && bC && !bR) {
        // (0 1 0) على الخط بالظبط -> امشي مستقيم
        lastLineDir = 0;
        driveLR(BASE_SPEED, +1, BASE_SPEED, +1);
      } else if (bL && bC && !bR) {
        // (1 1 0) انحراف بسيط للشمال -> هدّي العجلة الشمال شوية
        lastLineDir = -1;
        driveLR(TURN_SLOW, +1, BASE_SPEED, +1);
      } else if (bL && !bC && !bR) {
        // (1 0 0) انحراف قوي للشمال -> لف شمال
        lastLineDir = -1;
        driveLR(0.0f, 0, TURN_FAST, +1);
      } else if (!bL && bC && bR) {
        // (0 1 1) انحراف بسيط لليمين -> هدّي العجلة اليمين شوية
        lastLineDir = +1;
        driveLR(BASE_SPEED, +1, TURN_SLOW, +1);
      } else if (!bL && !bC && bR) {
        // (0 0 1) انحراف قوي لليمين -> لف يمين
        lastLineDir = +1;
        driveLR(TURN_FAST, +1, 0.0f, 0);
      } else if (!bL && !bC && !bR) {
        // (0 0 0) الخط ضاع لحظياً -> دور في اتجاه آخر حساس شاف الخط
        if (lastLineDir < 0) {
          driveLR(SEARCH_SPD, -1, SEARCH_SPD, +1); // لف شمال في مكانه للرجوع للخط
        } else if (lastLineDir > 0) {
          driveLR(SEARCH_SPD, +1, SEARCH_SPD, -1); // لف يمين في مكانه للرجوع للخط
        } else {
          driveLR(SEARCH_SPD, +1, SEARCH_SPD, +1);
        }
      }
      break;
    }

    case STATE_OBSTACLE_WAIT: {
      motorsStop();
      // نقيس المسافة يمين وشمال بانتظام، وأول ما الطريق يفضى نكمل
      distRightCm = readUltrasonicMedian(TRIG_R, ECHO_R);
      distLeftCm  = readUltrasonicMedian(TRIG_L, ECHO_L);
      if (distRightCm >= SAFE_GO_CM && distLeftCm >= SAFE_GO_CM) {
        digitalWrite(LED_PIN, LOW);
        Serial.println(F("[SAFETY CLEAR] Path clear! Resuming mission..."));
        delay(300); // وقفة أمان قصيرة قبل استئناف الحركة
        state = prevStateBeforeObstacle;
      }
      break;
    }

    case STATE_STATION_DROP:
      // الروبوت وصل محطة التسليم: يقف تماماً وينزل كيس الشيبسي بهدوء (90° -> 0°)
      motorsStop();
      delay(400);
      if (carryingChipsy) {
        Serial.println(F("[STATE] Lowering Chipsy bag at destination (90 -> 0 deg)..."));
        servoMoveSmooth(90, 0, 20);
        carryingChipsy = false;
      }
      Serial.println(F("[MISSION COMPLETE] Chipsy delivered! Waiting in IDLE (Press BOOT for next run)."));
      blinkLED(5, 120, 120);
      state = STATE_IDLE;
      break;
  }
}
