// Open-loop test of the DC motor through an L298N driver.
// A potentiometer on A0 sets the speed (PWM duty cycle) and a push button
// on pin 4 toggles the rotation direction.

#define enA 9      // L298N ENA: PWM speed input
#define in1 6      // L298N IN1 \ direction inputs:
#define in2 7      // L298N IN2 / (HIGH,LOW) = one way, (LOW,HIGH) = the other
#define button 4  // direction toggle button

int rotDirection = 0;  // current direction: 0 or 1
int pressed = false;   // toggled on every button press

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(button, INPUT);
  // Set initial rotation direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void loop() {
  int potValue = analogRead(A0); // Read potentiometer value
  int pwmOutput = map(potValue, 0, 1023, 0 , 255); // Map the potentiometer value from 0 to 255
  analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin

  // Read button - Debounce
  if (digitalRead(button) == true) {
    pressed = !pressed;
  }
  while (digitalRead(button) == true);  // wait until the button is released
  delay(20);

  // If button is pressed - change rotation direction
  if (pressed == true  & rotDirection == 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    rotDirection = 1;
    delay(20);
  }
  // If button is pressed - change rotation direction
  if (pressed == false & rotDirection == 1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    rotDirection = 0;
    delay(20);
  }
}
