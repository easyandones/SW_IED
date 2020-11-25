#include <Servo.h>
Servo myservo;

// Arduino pin assignment
#define PIN_IR A0
#define PIN_SERVO 10

#define SERVO_MAX 2300
#define SERVO_MIN 700

#define _INTERVAL_DIST 30
#define DELAY_MICROS  1500
#define EMA_ALPHA 0.35

const float coE[] = {0.0000006, -0.0013340, 1.4066328, 5.5307588};

float ema_dist = 0;
float filtered_dist;
float samples_num = 3;

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

void loop() {
  float dist = filtered_ir_distance();
  Serial.print("min:0,max:500,dist:");
  Serial.println(dist);
  if(dist < 255) {
    myservo.writeMicroseconds(SERVO_MAX);
  }
  else {
    myservo.writeMicroseconds(SERVO_MIN);
  }
  delay(20);
}
