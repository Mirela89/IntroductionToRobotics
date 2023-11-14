// DS = [D]ata [S]torage - data
// STCP = [ST]orage [C]lock [P]in latch
// SHCP = [SH]ift register [C]lock [P]in clock

const byte latchPin = 11; // STCP to 12 on Shift Register
const byte clockPin = 10; // SHCP to 11 on Shift Register
const byte dataPin = 12; // DS to 14 on Shift Register

// Pin assignments for controlling the common cathode/anode pins of the 7-segment digits
const byte segD1 = 4;
const byte segD2 = 5;
const byte segD3 = 6;
const byte segD4 = 7;

// Size of the register in bits
const byte regSize = 8;

// Array to keep track of the digit control pins
byte displayDigits[] = {
  segD1, segD2, segD3, segD4
};
const byte displayCount = 4; // Number of digits in the display
const int encodingsNumber = 10; // Number of different character encodings

byte byteEncodings[encodingsNumber] = {
  // Encoding for segments A through G and the decimal point (DP)
  //A B C D E F G DP
  B11111100, // 0
  B01100000, // 1
  B11011010, // 2
  B11110010, // 3
  B01100110, // 4
  B10110110, // 5
  B10111110, // 6
  B11100000, // 7
  B11111110, // 8
  B11110110 // 9
};

// Array representing the state of each bit in the shift register
byte registers[regSize];

//Variables for the buttons
const int button1Pin = 3; // START/PAUSE
const int button2Pin = 2; // RESET
const int button3Pin = 8; // SAVE LAP

byte button1State = 0; 
byte button2State = 0;
byte button3State = 0; 

// Stopwatch variables
unsigned long elapsedTime = 0;
bool isRunning = false;
unsigned long lastLapTime = 0;


const int maxLaps = 4; // Maximum number of laps to store
unsigned long lapTimes[maxLaps]; // Array to store lap times
int lapIndex = 0; // Index to keep track of the current lap

unsigned long lastIncrement = 0;
unsigned long delayCount = 100; // Delay between updates (milliseconds)
unsigned long number = 0; // The number being displayed

// Variables for button debouncing
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50; 

unsigned long lastButton2Time = 0;
unsigned long button2DebounceDelay = 50;

unsigned long lastButton3Time = 0;
unsigned long button3DebounceDelay = 50;

// New flag to track if the stopwatch is paused
bool isPaused = false;


void setup() {
  // Initialize the digital pins connected to the shift register as outputs
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  // Initialize the digit control pins as outputs and turn them off
  for (byte i = 0; i < displayCount; i++) {
    pinMode(displayDigits[i], OUTPUT);
    digitalWrite(displayDigits[i], LOW);
  }

  // Initialize the button pins
  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
  pinMode(button3Pin, INPUT_PULLUP);


  Serial.begin(9600);
}

void loop() {
  // Read button states
  button1State = digitalRead(button1Pin);
  button2State = digitalRead(button2Pin);
  button3State = digitalRead(button3Pin);

  // Handle button actions
  if (button1State == LOW) { // Start/Stop button
    // Check if enough time has passed since the last button press
    if (millis() - lastDebounceTime > debounceDelay) {
        lastDebounceTime = millis();  // Save the current time

        if (isRunning) {
            isRunning = false; // stop the timer
        } else {
            isRunning = true;
        }
    }
  }

  // If you press the reset button while timer works, nothing happens.
  if (!isRunning && button2State == LOW) { // Reset button
    // Check if enough time has passed since the last button press
    if (millis() - lastButton2Time > button2DebounceDelay) {
      lastButton2Time = millis();  // Save the current time
      isRunning = false;
      number = 0; // Reset the displayed number to 0
    }
    updateDisplay(number);
  }

  // Handle Lap button when the stopwatch is not paused
  if (!isPaused && button3State == LOW) {
    // Check if enough time has passed since the last button press
    if (millis() - lastButton3Time > button3DebounceDelay) {
      lastButton3Time = millis();  // Save the current time

      unsigned long currentMillis = millis();
      unsigned long lapTime = currentMillis - lastLapTime;

      Serial.print("Lap Time: ");
      printLapTime(lapTime);
      lastLapTime = currentMillis;

      if (isRunning) {
        // Save lap time only if the timer is running
        if (lapIndex < maxLaps) {
          lapTimes[lapIndex] = elapsedTime;
          lapIndex++;
        } else {
          // Override the 1st lap time with the latest one if max laps reached
          lapTimes[0] = elapsedTime;
        }
      }
    }
  }


  if (!isPaused) {
    // Only update the number and display if the stopwatch is not paused
    if (isRunning) {
      if (millis() - lastIncrement > delayCount) {
        number++;
        number %= 10000;
        lastIncrement = millis();
      }
    }
    updateDisplay(number);
  }
}


void updateDisplay(unsigned long number) {

  for (int i = 0; i < displayCount; i++) {
    activateDisplay(i);
    writeReg(B00000000); // Clear the register to avoid ghosting
  }

  //Initialize necessary variables for tracking the current number and digit position
  int currentNumber = number;
  int displayDigit = 3; // Start with the least significant digit
  int lastDigit = 0;

  //Loop through each digit of the current number
  while (displayDigit != -1) {
    lastDigit = currentNumber % 10; //Extract the last digit of the current number
    if(displayDigit == 2){
      activateDisplay(displayDigit); 
      writeReg(byteEncodings[lastDigit] + 1);
    } else { 
      activateDisplay(displayDigit); 
      writeReg(byteEncodings[lastDigit]); 
    }
    delay(0); 
    displayDigit--;
    currentNumber /= 10;
    writeReg(B00000000); // Clear the register to avoid ghosting
  }
}



void printLapTime(unsigned long lapTime) {
  // Convert lap time to the format you want to print and print it to Serial
  unsigned int seconds = lapTime / 1000;
  unsigned int tenths = (lapTime % 1000) / 100;

  Serial.print(seconds);
  Serial.print(".");
  Serial.println(tenths);
}


// Function to output a byte to the shift register
void writeReg(int encoding) {
  digitalWrite(latchPin, LOW); // Pull latch low to start data transfer
  // Shift out the bits of the 'encoding' to the shift register
  shiftOut(dataPin, clockPin, MSBFIRST, encoding); // MSBFIRST means the most significant bit is shifted out first
  // Pull latch high to transfer data from shift register to storage registernt
  digitalWrite(latchPin, HIGH); // This action updates the output of the shift register
}


void activateDisplay(int displayNumber) {
// 1. Deactivate all digit control pins to prevent ghosting on the display.
for (int i = 0; i < displayCount; i++) {
digitalWrite(displayDigits[i], HIGH);
}
// 2. Activate only the digit corresponding to 'displayNumber' parameter.
// This will be used to control which digit is illuminated on a 7-segment display.
digitalWrite(displayDigits[displayNumber], LOW);
}