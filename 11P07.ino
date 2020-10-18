#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 180 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360 // maximum distance to be measured (unit: mm)

#define _DUTY_MIN 550 // servo full clockwise position (0 degree)
#define _DUTY_MAX 2400 // servo full counterclockwise position (180 degree)
#define _DUTY_NEU (_DUTY_MIN+_DUTY_MAX)/2 // servo neutral position (90 degree)
#define _DUTY_RATIO float(_DUTY_MAX-_DUTY_MIN)/180

#define RES_LEN 5

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw, dist_prev, dist_ema; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
Servo myservo;

float results[RES_LEN] = {0,};

void save_data(float data) {
  for (int i = 0; i < RES_LEN - 1; i++) {
    results[i] = results[i + 1];
  }
  results[RES_LEN - 1] = data;
}

float get_middle(float sort_array[]) {
  float new_array[RES_LEN];
  for (int i = 0; i < RES_LEN; i++) {
    new_array[i] = sort_array[i];
  }
  int temp;
  for (int i = 0; i < RES_LEN; i++) {
    for (int j = 0; j < RES_LEN - 1; j++) {
      if (new_array[j] > new_array[j + 1]) {
        temp = new_array[j];
        new_array[j] = new_array[j + 1];
        new_array[j + 1] = temp;
      }
    }
  }
  return new_array[RES_LEN / 2];
}

void move_servo(Servo servo, float angle) {
  servo.writeMicroseconds(_DUTY_MIN+_DUTY_RATIO*angle);
}

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW); 
  pinMode(PIN_ECHO,INPUT);

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = dist_prev = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;

// initialize serial port
  Serial.begin(57600);

// initialize last sampling time
  last_sampling_time = 0;
}

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

// get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);
  save_data(dist_raw);
  dist_ema = get_middle(results);

// output the read value to the serial port
  Serial.print("Min:100,raw:");
  Serial.print(dist_raw);
  Serial.print(",ema:");
  Serial.print(dist_ema);
  Serial.print(",servo:");
  Serial.print(myservo.read());  
  Serial.println(",Max:300");

// adjust servo position according to the USS read value

  // add your code here!
  move_servo(myservo, dist_ema-180);
   
// update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  reading = pulseIn(ECHO, HIGH, timeout) * scale; // unit: mm
  if(reading < dist_min || reading > dist_max) {
    reading = 0.0; // return 0 when out of range.
    digitalWrite(PIN_LED, HIGH);
  }
  else {
    digitalWrite(PIN_LED, LOW);
  }

  if(reading == 0.0) reading = dist_prev;
  else dist_prev = reading;
  
  return reading;
}
