#define PIN_LED 7
unsigned int a, count;
void setup() {
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 0);
  delay(1000);
  int count = a = 0;
}

void loop() {
    for (a = 0; a<=10; a++) {
      count = 1 - count;
      digitalWrite(PIN_LED, count);
      delay(100);
      if (a==10) {
        while(1) {
          digitalWrite(PIN_LED, 1);
        }
      }
      
    }
} 
