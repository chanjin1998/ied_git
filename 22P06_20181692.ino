#include <Servo.h>
Servo myservo;
/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9                            
#define PIN_SERVO 10    
#define PIN_IR A0    

// Framework setting
#define _DIST_TARGET 255  
#define _DIST_MIN 100                     
#define _DIST_MAX 410  
// Distance sensor
#define _DIST_ALPHA 0.3  

// Servo range
#define _DUTY_MIN 1150  
#define _DUTY_NEU 1520     
#define _DUTY_MAX 1860                

// Servo speed control
#define _SERVO_ANGLE 90   

#define _SERVO_SPEED 60           

// Event periods
#define _INTERVAL_DIST 20   
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100  

// PID parameters
#define KP 1

int a = 70;
int b = 290;
//////////////////////
// global variables //
//////////////////////
float dist_min, dist_max;
// Servo instance
  
// Distance sensor
float dist_target; // location to send the ball 
float dist_raw, dist_ema;    

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial; 

// Servo speed control
int duty_chg_per_interval;   
int duty_target, duty_curr;    

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm; 
int dist_filtered;
//
unsigned long last_sampling_time; // unit: ms
int toggle_interval, toggle_interval_cnt;
void setup() {
// initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED, OUTPUT);  
  myservo.attach(PIN_SERVO);  
  myservo.writeMicroseconds(1520);
// initialize global variables
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;  
  dist_target = _DIST_TARGET; 


// move servo to neutral position
  myservo.writeMicroseconds(_DUTY_MIN);
// initialize serial port
    Serial.begin(57600);                          
// convert angle speed into duty change per interval.
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;          
}

float ir_distance_filtered(void){ // return value unit: mm
  return ir_distance(); // for now, just use ir_distance() without noise filter.
}


void loop() {
  float dist_raw = ir_distance();
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


////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
     event_dist = false;
  // get a distance reading from the distance sensor
     dist_ema = _DIST_ALPHA * dist_raw + (1 - _DIST_ALPHA) * dist_ema;
     dist_filtered = ir_distance_filtered();  


  // PID control logic
    error_curr = _DIST_TARGET - dist_filtered;  
    pterm = KP*error_curr; 
    control = pterm;           

  // duty_target = f(duty_neutral, control)
     duty_target = _DUTY_NEU + control;

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target < _DUTY_MIN) {
      duty_target = _DUTY_MIN;  // duty_target < _DUTY_MIN 일 때 duty_target 를 _DUTY_MIN 로 고정
      myservo.writeMicroseconds(duty_target);
    } else if (duty_target > _DUTY_MAX) {
       duty_target = _DUTY_MAX; // duty_target > _DUTY_MAX 일 때 duty_target 를 _DUTY_MAX 로 고정
       myservo.writeMicroseconds(duty_target); 
    }  

  }
  
  
  if(event_servo) {
    event_servo = false; // [3153] servo EventHandler Ticket -> false
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target; 
    }  
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }       
    myservo.writeMicroseconds(duty_target);
  }

  float raw_dist = ir_distance();
  float dist_cali = 160 + 300.0 / (b - a) * (raw_dist - a);
  
  if(event_serial) {
    event_serial = false; // [3153] serial EventHandler Ticket -> false
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }
}
