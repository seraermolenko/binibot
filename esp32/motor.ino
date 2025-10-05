// ESP32-C6 + TB6612FNG (UART from Raspberry Pi) — Arduino-ESP32 v3 API
// Pins 2..7 for motors, STBY tied to 3V3. UART1 on IO16/IO17.
// Sends/receives "V <v> <w>\n" and "STOP\n". Also supports WASD on USB Serial.

#include <Arduino.h>

// ----- TB6612 pins -----
#define AIN1 2
#define AIN2 3
#define PWMA 4
#define BIN1 5
#define BIN2 6
#define PWMB 7
// STBY: wire to 3V3 externally for this test (or drive with a pin later)

// ----- UART from Pi (UART1) -----
#define RX1 16   // Pi TXD0 -> ESP32 IO16
#define TX1 17   // Pi RXD0 <- ESP32 IO17
HardwareSerial PiSer(1);  // use UART1



//LED Boot UP

#ifndef LED_PIN
#define LED_PIN 15        // safe spare pin on your C6; change if your board has a real LED_BUILTIN
#endif
#ifndef LED_ACTIVE_LOW
#define LED_ACTIVE_LOW 0  // set to 1 if your LED turns ON when the pin is LOW
#endif

inline void ledOn()  { digitalWrite(LED_PIN, LED_ACTIVE_LOW ? LOW  : HIGH); }
inline void ledOff() { digitalWrite(LED_PIN, LED_ACTIVE_LOW ? HIGH : LOW ); }

void blinkBoot(int times=3, int on_ms=120, int off_ms=120){
  for(int i=0;i<times;i++){ ledOn(); delay(on_ms); ledOff(); delay(off_ms); }
}

// ----- Config -----
const uint32_t PWM_FREQ = 20000;   // 20 kHz (quiet)
const uint8_t  PWM_BITS = 8;       // duty 0..255
int            DUTY_MAX = 200;     // cap motor power (tune 160..255)
const int      HEARTBEAT_MS = 1000;

const bool INVERT_TURN  = true;    // set true if your "turn right" goes left
const bool INVERT_LEFT  = false;   // flip a side in software if wiring is reversed
const bool INVERT_RIGHT = false;

const int   DB_LEFT  = 30;         // deadband nudge so the side starts reliably
const int   DB_RIGHT = 40;
const float GAIN_LEFT  = 1.00;     // fine trim between sides
const float GAIN_RIGHT = 1.05;

// ----- Helpers -----
static inline void setSide(int in1, int in2, int pwmPin, int duty, bool fwd, bool invertSide){
  bool dir = fwd ^ invertSide;
  digitalWrite(in1, dir ? HIGH : LOW);
  digitalWrite(in2, dir ? LOW  : HIGH);
  ledcWrite(pwmPin, duty);
}

void mixAndDrive(float v, float w){
  if (INVERT_TURN) w = -w;

  float Lf = constrain(v - w, -1.f, 1.f);
  float Rf = constrain(v + w, -1.f, 1.f);

  int L = (int)(fabs(Lf) * DUTY_MAX * GAIN_LEFT);
  int R = (int)(fabs(Rf) * DUTY_MAX * GAIN_RIGHT);
  if (L > 0) L = constrain(DB_LEFT  + L, 0, 255);
  if (R > 0) R = constrain(DB_RIGHT + R, 0, 255);

  setSide(AIN1, AIN2, PWMA, L, (Lf >= 0), INVERT_LEFT);
  setSide(BIN1, BIN2, PWMB, R, (Rf >= 0), INVERT_RIGHT);
}

bool readLineFrom(Stream& S, String& out){
  if (!S.available()) return false;
  out = S.readStringUntil('\n');
  out.trim();
  return out.length() > 0;
}

// ----- State -----
unsigned long lastCmdMs = 0;
float v_now = 0, w_now = 0;

void setup(){
  delay(200);


 // LED init + boot blink
  pinMode(LED_PIN, OUTPUT);
  ledOff();
  blinkBoot(3, 120, 120);  // 3 quick blinks on boot


  // Debug over USB (optional)
  Serial.begin(115200);
  Serial.setTimeout(10);

  // UART1 from Pi on IO16/IO17
  PiSer.begin(115200, SERIAL_8N1, RX1, TX1);
  PiSer.setTimeout(10);




  pinMode(AIN1,OUTPUT); pinMode(AIN2,OUTPUT);
  pinMode(BIN1,OUTPUT); pinMode(BIN2,OUTPUT);

  // Attach PWM (v3 API)
  ledcAttach(PWMA, PWM_FREQ, PWM_BITS);
  ledcAttach(PWMB, PWM_FREQ, PWM_BITS);
  ledcWrite(PWMA, 0); ledcWrite(PWMB, 0);

  Serial.println("ESP32-C6 motor ctrl (UART1 on IO16/17) ready");
  lastCmdMs = millis();
}

void handleKey(char k){            // WASD from USB Serial for manual test
  const float V_STEP=0.10, W_STEP=0.15;
  switch (k){
    case 'w': v_now = min( 0.6f, v_now + V_STEP); w_now = 0; break;
    case 'x': v_now = max(-0.4f, v_now - V_STEP); w_now = 0; break;
    case 'a': w_now = max(-0.8f, w_now - W_STEP); v_now = 0; break;
    case 'd': w_now = min( 0.8f, w_now + W_STEP); v_now = 0; break;
    case 's': v_now = 0; w_now = 0; break;
    case '+': DUTY_MAX = min(255, DUTY_MAX+10); break;
    case '-': DUTY_MAX = max(80,  DUTY_MAX-10); break;
    default: return;
  }
  mixAndDrive(v_now, w_now);
  lastCmdMs = millis();
  Serial.printf("MAN v=%.2f w=%.2f duty=%d\n", v_now, w_now, DUTY_MAX);
}

void loop(){
  // Prefer commands from Pi (UART1), fall back to USB Serial
  String cmd;
  if (!readLineFrom(PiSer, cmd)) {
    if (Serial.available()){
      int c = Serial.peek();
      if (c=='V' || c=='S') { readLineFrom(Serial, cmd); }
      else { handleKey(Serial.read()); }     // single-key WASD
    }
  }

  if (cmd.length()){
    if (cmd.startsWith("V ")){
      float v,w; if (sscanf(cmd.c_str(),"V %f %f",&v,&w)==2){
        v_now = constrain(v, -1.f, 1.f);
        w_now = constrain(w, -1.f, 1.f);
        mixAndDrive(v_now, w_now);
        lastCmdMs = millis();
      }
    } else if (cmd=="STOP"){
      ledcWrite(PWMA,0); ledcWrite(PWMB,0);
      v_now = w_now = 0;
      lastCmdMs = millis();
    }
  }

  // Heartbeat timeout → stop
  if (millis() - lastCmdMs > HEARTBEAT_MS){
    ledcWrite(PWMA,0); ledcWrite(PWMB,0);
    v_now = w_now = 0;
  }
}
