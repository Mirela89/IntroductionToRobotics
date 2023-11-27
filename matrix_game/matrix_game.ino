#include "LedControl.h" // Include LedControl library for controlling the LED matrix

const int dinPin = 12;
const int clockPin = 11;
const int loadPin = 10;

const int joystickButtonPin = 2;
const int xPin = A0;
const int yPin = A1;

// Create an LedControl object to manage the LED matrix
LedControl lc = LedControl(dinPin, clockPin, loadPin, 1); // DIN, CLK, LOAD, No. DRIVER

// Variable to set the brightness level of the matrix
byte matrixBrightness = 2;

// Variables to track the current and previous positions of the joystick-controlled LED
byte xPos = 0;
byte yPos = 0;
byte xLastPos = 0;
byte yLastPos = 0;

// Thresholds for detecting joystick movement
const int minThreshold = 200;
const int maxThreshold = 600;

const byte moveInterval = 200; // Timing variable to control the speed of LED movement
unsigned long long lastMoved = 0; // Tracks the last time the LED moved
const byte blinkInterval = 1000; // Blinking interval 
unsigned long lastBlink = 0; // Tracks the last time the LED blinked

const byte matrixSize = 8 ;// Size of the LED matrix
bool matrixChanged = true; // Flag to track if the matrix display needs updating

const byte wallPercentage = 50; // Percentage of the map covered by walls
const byte bombBlinkInterval = 500; // Blinking interval for the bomb

const int debounceDelay = 50; // Adjust this value as needed
unsigned long lastButtonPress = 0;


// 2D array representing the state of each LED (on/off) in the matrix
byte matrix[matrixSize][matrixSize] = {
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0}
};


void setup() {
  Serial.begin(9600);
  // the zero refers to the MAX7219 number, it is zero for 1 chip
  lc.shutdown(0, false); // turn off power saving, enables display
  lc.setIntensity(0, matrixBrightness); // sets brightness (0~15 possible values)
  lc.clearDisplay(0); // Clear the matrix display

  pinMode(joystickButtonPin, INPUT_PULLUP); // activate pull-up resistor on the 

  initializeWalls(); // Initialize walls on the map
  matrix[xPos][yPos] = 1; // Initialize the starting position of the LED
}


void loop() {
  // Check if it's time to move the LED
  if (millis() - lastMoved > moveInterval) { 
    updatePositions(); // Update the position of the LED based on joystick input
    lastMoved = millis(); // Update the time of the last move
  }

  if (millis() - lastBlink >= blinkInterval) {
    lastBlink = millis();
    matrix[xPos][yPos] = !matrix[xPos][yPos]; // Toggle the LED state
    matrixChanged = true; // Set the flag to update the matrix
  }

  if (digitalRead(joystickButtonPin) == LOW && millis() - lastButtonPress > debounceDelay) {
    lastButtonPress = millis();
    placeBomb(); // Place a bomb and destroy nearby walls
  }

  // Check if the matrix display needs updating
  if (matrixChanged == true) { 
    updateMatrix(); // Update the LED matrix display
    matrixChanged = false; // Reset the update flag
  }
}

void updateMatrix() {
  for (int row = 0; row < matrixSize; row++) {
    for (int col = 0; col < matrixSize; col++) {
      lc.setLed(0, row, col, matrix[row][col]); // set each led individually
    }
  }
}

// Function to read joystick input and update the position of the LED
void updatePositions() {
  int xValue = analogRead(xPin);
  int yValue = analogRead(yPin);

  // Store the next positions of the LED
  int nextXPos = xPos;
  int nextYPos = yPos;

  // Update nextXPos based on joystick movement (X-axis)
  if (xValue < minThreshold) {
    nextXPos = (xPos < matrixSize - 1) ? (xPos + 1) : xPos;
  }

  if (xValue > maxThreshold) {
    nextXPos = (xPos > 0) ? (xPos - 1) : xPos;
  }

  // Update nextYPos based on joystick movement (Y-axis)
  if (yValue > maxThreshold) {
    nextYPos = (yPos < matrixSize - 1) ? (yPos + 1) : yPos;
  }

  if (yValue < minThreshold) {
    nextYPos = (yPos > 0) ? (yPos - 1) : yPos;
  }

  // Check if the next position is not a wall
  if (matrix[nextXPos][nextYPos] != 1) {
    // Update the matrix if the position has changed
    if (nextXPos != xPos || nextYPos != yPos) {
      matrixChanged = true;
      matrix[xPos][yPos] = 0; // Clear the current position
      matrix[nextXPos][nextYPos] = 1; // Set the LED at the new position
      xPos = nextXPos;
      yPos = nextYPos;
    }
  }
}

void initializeWalls() {
  // Use analogRead(0) as a seed for randomness
  randomSeed(analogRead(0));

  for (int row = 0; row < matrixSize; row++) {
    for (int col = 0; col < matrixSize; col++) {
      // Skip the player's initial position
      if (row == xPos && col == yPos) {
        matrix[row][col] = 0; // Leave the space empty for the player
      } else {
        // Generate random number to determine if a wall should be placed
        if (random(100) < wallPercentage) {
          matrix[row][col] = 1; // Place a wall
        } else {
          matrix[row][col] = 0; // Leave the space empty
        }
      }
    }
  }
}


void placeBomb() {
  // Set the bomb position and initialize the bomb timer
  int bombX = xPos;
  int bombY = yPos;
  unsigned long bombStartTime = millis();
  unsigned long lastBlinkTime = bombStartTime;

  // Blink the bomb for a certain duration
  while (millis() - bombStartTime < bombBlinkInterval * 4) {
    if (millis() - lastBlinkTime >= bombBlinkInterval) {
      lastBlinkTime = millis();
      matrix[bombX][bombY] = !matrix[bombX][bombY]; // Toggle the bomb LED state
      updateMatrix(); // Update the LED matrix display
    }
  }

  // Destroy nearby walls on the horizontal axis
  for (int i = -2; i <= 2; i++) {
    int destroyX = bombX + i;
    if (destroyX >= 0 && destroyX < matrixSize && matrix[destroyX][bombY] == 1) {
      matrix[destroyX][bombY] = 0; // Destroy the wall
    }
  }

  // Destroy nearby walls on the vertical axis
  for (int j = -1; j <= 1; j++) {
    int destroyY = bombY + j;
    if (destroyY >= 0 && destroyY < matrixSize && matrix[bombX][destroyY] == 1) {
      matrix[bombX][destroyY] = 0; // Destroy the wall
    }
  }
}
