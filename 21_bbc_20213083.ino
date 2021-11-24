#include <Servo.h>

#define PIN_IR A0

// Arduino pin assignment
#define PIN_SERVO 10

// configurable parameters
#define _DUTY_MIN 1210 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1481 // servo neutral position (90 degree)
#define _DUTY_MAX 1752 // servo full counterclockwise position (180 degree)
#define _INTERVAL_DIST    25 // USS interval (unit: ms)
#define _INTERVAL_SERVO    20 // servo interval (unit: ms)
#define _INTERVAL_SERIAL    100 // serial interval (unit: ms)
#define _DIST_ALPHA 0.5

#define _POS_START 1481

#define _SERVO_SPEED 80 // servo speed limit (unit: degree/second)
#define INTERVAL 20  // servo update interval

// global variables
unsigned long last_sampling_time; // unit: ms
int duty_chg_per_interval; // maximum duty difference per interval
int toggle_interval, toggle_interval_cnt;
float pause_time; // unit: sec
Servo myservo;
int duty_target, duty_curr; 
float alpha, dist_prev, dist_ema;
float timeout;
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; // unit: ms
bool event_dist, event_servo, event_serial;
float raw_dist, dist_cali;

void setup() {
// initialize GPIO pins
  myservo.attach(PIN_SERVO); 
  duty_target = duty_curr = 0;
  myservo.write(90);
  duty_curr = duty_target = 1481;
// initialize serial port
  Serial.begin(57600);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (float)(1846) * _SERVO_SPEED / 180 * INTERVAL / 1000;
  //duty_chg_per..을 float로 선언하고 보았을 때 속도가 4이면 값이 0.82가 된다.
//초마다 변하는 서보의 각도  

// initialize variables for servo update.
  pause_time = 1;
  toggle_interval = (180.0 / _SERVO_SPEED + pause_time) * 1000 / INTERVAL;
  toggle_interval_cnt = toggle_interval;
  
// initialize last sampling time
  last_sampling_time = 0;
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  dist_prev = 0.0;
}

void loop() {
    unsigned long time_curr = millis();
    if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }

  if(event_dist) {
    event_dist = false;
    raw_dist = ir_distance();
    dist_cali = 150 + 200.0 / 238 * (raw_dist - 150);
    if(dist_cali < 100 || dist_cali > 450) dist_cali = 0.0; // return 0 when out of range.
    if(dist_cali == 0.0) dist_cali = dist_prev;
    else dist_prev = dist_cali;
    
    dist_ema = alpha * dist_cali + (1 - alpha) * dist_ema;

    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }

 
    //  Serial.println(duty_chg_per_interval);
    
    // toggle duty_target between _DUTY_MIN and _DUTY_MAX.

 
    if (dist_cali < 200.0) duty_target = 1752;
    else if (dist_cali < 250.0) duty_target = 1490;
    else duty_target = 1210;
  }

  if(event_servo) {
    event_servo = false;
    myservo.writeMicroseconds(duty_curr);
  }
  
// output the read value to the serial port
  if(event_serial) {
    event_serial = false;
    Serial.print("Min:1210,duty_target:");
    Serial.print(duty_target);
    Serial.print(",duty_curr:");
    Serial.print(duty_curr);
    Serial.println(",Max:1752");
    Serial.print(dist_cali);
    Serial.print(" dist_ema:");
    Serial.print(dist_ema);

  }
 
  

// update last sampling time
  last_sampling_time += INTERVAL;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}
