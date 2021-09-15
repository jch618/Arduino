#define PIN_LED 7
unsigned int count = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  delay(1000);
  for (int i = 0; i < 5; i++){
    digitalWrite(PIN_LED, 1);
    delay(100);
    digitalWrite(PIN_LED, 0);
    delay(100);  
  }
  digitalWrite(PIN_LED, 1);
  while(1){}
}
