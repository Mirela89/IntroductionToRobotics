// Declare all the joystick pins
const int pinSW = 2; // Digital pin connected to switch output
const int pinX = A0; // A0 - Analog pin connected to X output
const int pinY = A1; // A1 - Analog pin connected to Y output

// Declare all the segments pins
const int pinA = 12;
const int pinB = 10;
const int pinC = 9;
const int pinD = 8;
const int pinE = 7;
const int pinF = 6;
const int pinG = 5;
const int pinDP = 4;

const int segSize = 8;
int index = 7; // Start at the decimal point position

bool commonAnode = false;
const int noOfDigits = 10;
byte state = HIGH;
byte dpState = LOW;
byte swState = LOW;
byte lastSwState = LOW;
int xValue = 0;
int yValue = 0;

bool joyMoved = false;
int digit = 0;

int segments[segSize] = {
  pinA, pinB, pinC, pinD, pinE, pinF, pinG, pinDP
};

unsigned long lastButtonPressTime = 0;
unsigned long longPressDelay = 1000;

void setup() {
  // Initialize pins
  for (int i = 0; i < segSize; i++) {
    pinMode(segments[i], OUTPUT);
  }

  pinMode(pinSW, INPUT_PULLUP);

  if (commonAnode == true) {
    state = !state;
  }
}

void loop() {
  // Read joystick values
  xValue = analogRead(pinX);
  yValue = analogRead(pinY);

  // Handle joystick movement
  handleJoystickMovement();

  // Handle button press
  handleButtonPress();

  // Update display based on the current position and segment states
  updateDisplay();
}


void handleJoystickMovement() {
  int xThreshold = 350;
  int yThreshold = 650;

  // Read joystick values and apply threshold to reduce noise
  int xChange = analogRead(pinX) - 512; 
  int yChange = analogRead(pinY) - 512; 

   // Switch based on the current position
  switch (index) {
    case 0: // a
      if (xChange == 1) { // RIGHT input
        index = 1; // go to b
      } else if (xChange == -1) { // LEFT input
        index = 5; // go to f
      } else if (yChange == -1) { // DOWN input
        index = 6; // go to g
      }
      break;

    case 1: // b
      if (yChange == 1) { // UP input
        index = 0; // go to a
      } else if (xChange == -1) { // LEFT input
        index = 5; // go to f
      } else if (yChange == -1) { // DOWN input
        index = 6; // go to g
      }
      break;

    case 2: // c
      if (xChange == -1) { // LEFT input
        index = 4; // go to e
      } else if (yChange == 1) { // UP input
        index = 6; // go to g
      } else if (yChange == -1) { // DOWN input
        index = 3; // go to d
      } else if (xChange == 1) { // RIGHT input
        index = 7; // go to dp
      }
      break;

    case 3: // d
      if (xChange == 1) { // RIGHT input
        index = 2; // go to c
      } else if (yChange == 1) { // UP input
        index = 6; // go to g
      } else if (xChange == -1) { // LEFT input
        index = 4; // go to e
      }
      break;

    case 4: // e
      if (xChange == 1) { // RIGHT input
        index = 2; // go to c
      } else if (yChange == -1) { // DOWN input
        index = 3; // go to d
      } else if (yChange == 1) { // UP input
        index = 6; // go to g
      }
      break;

    case 5: // f
      if (yChange == 1) { // UP input
        index = 0; // go to a
      } else if (yChange == -1) { // DOWN input
        index = 6; // go to g
      } else if (xChange == -1) { // RIGHT input
        index = 1; // go to b
      }
      break;

    case 6: // g
      if (yChange == 1) { // UP input
        index = 0; // go to a=5t
      } else if (yChange == -1) { // DOWN input
        index = 3; // go to d
      }
      break;

    case 7: // dp
      if (xChange == -1) { // LEFT input
        index = 2; // go to c
      }
    break;
  }

  // Reset the flag if there is no joystick movement
  if (abs(xChange) <= xThreshold && abs(yChange) <= yThreshold) {
    joyMoved = false;
  }
}


void handleButtonPress() {
  swState = digitalRead(pinSW);

  // Handle short button press to toggle segment state
  if (swState == LOW && lastSwState == HIGH) {
    // Toggle the state of the current segment
    if (digitalRead(segments[index]) == HIGH) {
      digitalWrite(segments[index], LOW);
    } else {
      digitalWrite(segments[index], HIGH);
    }
  }

  // Handle long button press to reset display
  if (swState == LOW && millis() - lastButtonPressTime >= longPressDelay) {
    // Reset the entire display
    for (int i = 0; i < segSize; i++) {
      digitalWrite(segments[i], LOW);
    }
    index = 7; // Move back to the decimal point
    lastButtonPressTime = millis(); // Reset the timer
  }

  lastSwState = swState;
}




void updateDisplay() {
  for (int i = 0; i < segSize; i++) {
    // Turn off all segments
    digitalWrite(segments[i], LOW);
  }

  // Make the current position blink
  if (millis() % 1000 < 500) {
    digitalWrite(segments[index], HIGH); // Turn on the current position
  } else {
    digitalWrite(segments[index], LOW); // Turn off the current position during the other half of the blink cycle
  }
}
