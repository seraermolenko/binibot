#include <Arduino.h>


#define AIN1 2
#define AIN2 3
#define PWMA 4
#define BIN1 5
#define BIN2 6
#define PWMB 7


#define RX1 16   // Pi TXD0 -> ESP32 RX1
#define TX1 17   // Pi RXD0 <- ESP32 TX1
HardwareSerial PiSer(1);           // UART1


#define TRIG_PIN 18                
#define ECHO_PIN  1                


const uint32_t PWM_FREQ = 20000;   // 20 kHz
const uint8_t  PWM_BITS = 8;       // duty 0..255
int            DUTY_MAX = 180;
const int      HEARTBEAT_MS = 1000;
const bool     INVERT_TURN  = true;
const bool     INVERT_LEFT  = false;
const bool     INVERT_RIGHT = false;
const int      DB_LEFT  = 30,  DB_RIGHT = 40;
const float    GAIN_LEFT = 1.00, GAIN_RIGHT = 1.05;


static inline void setSide(int in1,int in2,int pwm,int duty,bool fwd,bool inv){
  bool dir = fwd ^ inv;
  digitalWrite(in1, dir?HIGH:LOW);
  digitalWrite(in2, dir?LOW :HIGH);
  ledcWrite(pwm, duty);
}
void mixAndDrive(float v,float w){
  if (INVERT_TURN) w = -w;
  float Lf = constrain(v - w, -1.f, 1.f);
  float Rf = constrain(v + w, -1.f, 1.f);
  int L = (int)(fabs(Lf) * DUTY_MAX * GAIN_LEFT);
  int R = (int)(fabs(Rf) * DUTY_MAX * GAIN_RIGHT);
  if (L>0) L = constrain(DB_LEFT  + L, 0, 255);
  if (R>0) R = constrain(DB_RIGHT + R, 0, 255);
  setSide(AIN1,AIN2,PWMA,L,(Lf>=0),INVERT_LEFT);
  setSide(BIN1,BIN2,PWMB,R,(Rf>=0),INVERT_RIGHT);
}

bool readLineFrom(Stream& S, String& out){
  if (!S.available()) return false;
  out = S.readStringUntil('\n'); out.trim();
  return out.length()>0;
}

unsigned long lastCmdMs = 0;
float v_now=0, w_now=0;

float hcsr04_read_cm() {
  // Trigger 10us pulse
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);


  unsigned long us = pulseInLong(ECHO_PIN, HIGH, 25000UL);
  if (us == 0) return -1.0f;                 // timeout / no echo
  // distance cm = time(us) / 58.0 (round trip at ~20°C)
  float cm = us / 58.0f;
  if (cm < 2.0f || cm > 400.0f) return -1.0f; // out of typical range
  return cm;
}

void setup(){
  delay(150);
  // Debug over USB optional
  Serial.begin(115200);
  Serial.setTimeout(10);

  // Pi UART
  PiSer.begin(115200, SERIAL_8N1, RX1, TX1);
  PiSer.setTimeout(5);

  // Motors
  pinMode(AIN1,OUTPUT); pinMode(AIN2,OUTPUT);
  pinMode(BIN1,OUTPUT); pinMode(BIN2,OUTPUT);
  ledcAttach(PWMA, PWM_FREQ, PWM_BITS);
  ledcAttach(PWMB, PWM_FREQ, PWM_BITS);
  ledcWrite(PWMA,0); ledcWrite(PWMB,0);

  // Ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);       // ensure divider on ECHO!

  Serial.println("ESP32-C6 motors + ultrasonic ready");
  lastCmdMs = millis();
}

void handleKey(char k){ // manual WASD on USB Serial (optional)
  const float V_STEP=0.10, W_STEP=0.15;
  switch(k){
    case 'w': v_now=min(0.6f, v_now+V_STEP); w_now=0; break;
    case 'x': v_now=max(-0.4f, v_now-V_STEP); w_now=0; break;
    case 'a': w_now=max(-0.8f, w_now-W_STEP); v_now=0; break;
    case 'd': w_now=min( 0.8f, w_now+W_STEP); v_now=0; break;
    case 's': v_now=0; w_now=0; break;
  }
  mixAndDrive(v_now, w_now);
  lastCmdMs = millis();
}

void loop(){
  String cmd;
  if (!readLineFrom(PiSer, cmd)){
    if (Serial.available()){
      int c = Serial.peek();
      if (c=='V' || c=='S') readLineFrom(Serial, cmd);
      else handleKey(Serial.read());
    }
  }
  if (cmd.length()){
    if (cmd.startsWith("V ")){
      float v,w; if (sscanf(cmd.c_str(),"V %f %f",&v,&w)==2){
        v_now = constrain(v,-1.f,1.f); w_now = constrain(w,-1.f,1.f);
        mixAndDrive(v_now, w_now);
        lastCmdMs = millis();
      }
    } else if (cmd=="STOP"){
      ledcWrite(PWMA,0); ledcWrite(PWMB,0);
      v_now=w_now=0; lastCmdMs=millis();
    }
  }

  // ---- Heartbeat timeout ----
  if (millis() - lastCmdMs > HEARTBEAT_MS){
    ledcWrite(PWMA,0); ledcWrite(PWMB,0);
    v_now=w_now=0;
  }

  static unsigned long lastU = 0;
  if (millis() - lastU >= 100){            // every 100 ms
    lastU = millis();
    float cm = hcsr04_read_cm();
    // Send to Pi; keep it simple: "U <cm>"
    if (cm < 0) PiSer.println("U -1");
    else        PiSer.printf("U %.2f\n", cm);
  }
}