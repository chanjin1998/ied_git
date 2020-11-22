#include <Servo.h>
#define PIN_SERVO 10

Servo myservo;

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9

int a,b;

void setup() {
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(1520);
// initialize serial port
  Serial.begin(57600);

  a = 69.6;
  b = 440.5;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  if (raw_dist > 247) 
     myservo.writeMicroseconds(1200);
  if (raw_dist < 247)
     myservo.writeMicroseconds(1820);
  
}
