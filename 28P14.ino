#include <Servo.h>
Servo myservo;

// Arduino pin assignment
#define PIN_IR A0
#define PIN_SERVO 10

#define SERVO_MAX 2300
#define SERVO_MID 1520
#define SERVO_MIN 700
#define SERVO_SPEED 30

#define _DIST_TARGET 255

//Serial
#define _INTERVAL_SERIAL 100
unsigned long last_sampling_time_serial;

// PID Controls
#define P_RATE 0.3 //기본 P 비율 - 아래 UP, DOWN 비율에서 2차 변환
#define P_RATE_UP P_RATE * ((SERVO_MAX - SERVO_MID) / 100)
#define P_RATE_DOWN P_RATE * ((SERVO_MID - SERVO_MIN) / 100)

#define D_RATE 20.0 //기본 D 비율 - 아래 UP, DOWN 비율에서 2차 변환
#define D_RATE_UP D_RATE * ((SERVO_MAX - SERVO_MID) / 100)
#define D_RATE_DOWN D_RATE * ((SERVO_MID - SERVO_MIN) / 100)

#define I_RATE 0.001 //기본 I 비율 - 아래 UP, DOWN 비율에서 2차 변환
#define I_RATE_UP I_RATE * ((SERVO_MAX - SERVO_MID) / 100)
#define I_RATE_DOWN I_RATE * ((SERVO_MID - SERVO_MIN) / 100)

#define DELAY_MICROS 100
#define EMA_ALPHA 0.25

//Filter
const float coE[] = {-0.0000120, 0.0076651, -0.2529813, 84.6894326};

float ema_dist = 0;
float filtered_dist;
float samples_num = 3;


//Servo
float servo_target = SERVO_MID;
float servo_curr = servo_target;

float p_term = 0;
float d_term = 0;
float i_term = 0;
float dist_prev = 0;

void setup() {
  myservo.attach(PIN_SERVO);
  Serial.begin(57600);
}

float ir_distance(void){
  float volt = float(analogRead(PIN_IR));
  float val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  float dist_cali = coE[0] * pow(val, 3) + coE[1] * pow(val, 2) + coE[2] * val + coE[3];
  return dist_cali;
}

float under_noise_filter(void){
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;
  return ema_dist;
}

float get_i_term(float new_i_term) {
  float calc_i_term = i_term;
  calc_i_term += new_i_term;
  return calc_i_term;
}

void servoWrite(float servo) {
  if (servo > SERVO_MAX) {
    servo = SERVO_MAX;
  }
  if (servo < SERVO_MIN) {
    servo = SERVO_MIN;
  }
  servo_target = servo;
  if (servo_target > servo_curr + SERVO_SPEED) {
    servo_curr += SERVO_SPEED;
  }
  else if (servo_target < servo_curr - SERVO_SPEED) {
    servo_curr -= SERVO_SPEED;
  }
  else {
    servo_curr = servo_target;
  }
  myservo.writeMicroseconds(servo_curr);
}

void servoUp(float dist_rate) {
  p_term = P_RATE_UP * dist_rate;
  d_term = D_RATE_UP * (dist_rate - dist_prev);
  i_term = get_i_term(I_RATE_UP * dist_rate);
  float result = SERVO_MID + p_term + d_term + i_term;
  servoWrite(result);
}

void servoDown(float dist_rate) {
  p_term = P_RATE_DOWN * dist_rate;
  d_term = D_RATE_DOWN * (dist_rate - dist_prev);
  i_term = get_i_term(I_RATE_DOWN * dist_rate);
  float result = SERVO_MID + p_term + d_term + i_term;
  servoWrite(result);
}

void moveServo(float dist_rate) {
  if (dist_rate > 0) {
    servoDown(dist_rate);
  }
  else {
    servoUp(dist_rate);
  }
  dist_prev = dist_rate;
}

float getDistRate(float dist_from_target) {
  return dist_from_target / 155 * 100;
}

void showLogs(float dist, float duty_target, float duty_curr) {
  if (millis() < last_sampling_time_serial + _INTERVAL_SERIAL) {
    delay(10);
    return;
  }
  last_sampling_time_serial = millis();
  Serial.print("IR:");
  Serial.print(dist);
  Serial.print(",T:");
  Serial.print(_DIST_TARGET);
  Serial.print(",P:");
  Serial.print(map(p_term, -1000, 1000, 510, 610));
  Serial.print(",D:");
  Serial.print(map(d_term, -1000, 1000, 510, 610));
  Serial.print(",I:");
  Serial.print(map(i_term, -1000, 1000, 510, 610));
  Serial.print(",DTT:");
  Serial.print(map(duty_target, SERVO_MIN, SERVO_MAX, 410, 510));
  Serial.print(",DTC:");
  Serial.print(map(duty_curr, SERVO_MIN, SERVO_MAX, 410, 510));
  Serial.println(",-G:245,+G:265,m:0,M:800");
}

void loop() {
  float dist = filtered_ir_distance();
  float dist_from_target = _DIST_TARGET - dist;
  float dist_rate = getDistRate(dist_from_target);
  moveServo(dist_rate);
  showLogs(dist, servo_target, servo_curr);
}
