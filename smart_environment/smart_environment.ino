#include <EEPROM.h>

// Define pin numbers
const int trigPin = 9; // Trigger pin for HC-SR04
const int echoPin = 10; // Echo pin for HC-SR04

const int ldrPin = A0; // Analog pin for LDR
const int rgbLedPin = 9; // PWM pin for RGB LED

// Define RGB LED control pins
const int redPin = 6;
const int greenPin = 5;
const int bluePin = 3;

// Define variables
int interval = 5;  // Default interval in seconds
int ultrasonicThreshold = 100;  // Default ultrasonic threshold
int ldrThreshold = 500;  // Default LDR threshold
bool automaticMode = true; // Default automatic mode for RGB LED


const int MAX_LOG_ENTRIES = 10; // Constant for maximum logged entries

// Arrays to store logged data
int ultrasonicLog[MAX_LOG_ENTRIES];
int ldrLog[MAX_LOG_ENTRIES];
int logIndex = 0;

int ultrasonicValue = 0;
int ldrValue = 0;


void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ldrPin, INPUT);
  pinMode(rgbLedPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Load settings from EEPROM
  loadSettings();
}

void loop() {
  processMenu();  // Call the menu processing function
}

void processMenu() {
  Serial.println("\nMain Menu:");
  Serial.println("1. Sensor Settings");
  Serial.println("2. Reset Logger Data");
  Serial.println("3. System Status");
  Serial.println("4. RGB LED Control");

  int choice;
  Serial.print("Enter your choice (1-4)");
  while (!Serial.available()) {
    // Wait for user input
  }
  choice = Serial.parseInt();
  while (Serial.read() != '\n');

  switch (choice) {
    case 1:
      processSensorSettingsMenu();
      break;
    case 2:
      processResetLoggerDataMenu();
      break;
    case 3:
      processSystemStatusMenu();
      break;
    case 4:
      processRgbLedControlMenu();
      break;
    default:
      Serial.println("\nInvalid choice. Please enter a number between 1 and 4.");
  }
}

// Sensor Settings Menu function
void processSensorSettingsMenu() {
  Serial.println("\nSensor Settings Menu:");
  Serial.println("1. Sensors Sampling Interval");
  Serial.println("2. Ultrasonic Alert Threshold");
  Serial.println("3. LDR Alert Threshold");
  Serial.println("4. Back");

  int choice;
  Serial.print("Enter your choice (1-4)");
  while (!Serial.available()) {
    // Wait for user input
  }
  choice = Serial.parseInt();
  while (Serial.read() != '\n');

  Serial.print("\nReceived choice in Sensor Settings menu: ");
  Serial.println(choice);

  switch (choice) {
    case 1:
      // Sensors Sampling Interval
      do {
        Serial.print("Enter sampling interval (1-10 seconds): ");
        while (!Serial.available()) {
          // Wait for user input
        }
        interval = Serial.parseInt();
        while (Serial.read() != -1); // Consume any remaining characters

        if (interval >= 1 && interval <= 10) {
          Serial.print("\nSampling interval set to: ");
          Serial.print(interval);
          Serial.println(" seconds");
        } else {
        Serial.println("\nInvalid choice. Please enter a number between 1 and 10.");
        }
      } while (interval < 1 || interval > 10);
    break;

    case 2:
      // Ultrasonic Alert Threshold
      Serial.print("\nEnter ultrasonic threshold value: ");
      while (!Serial.available()) {
        // Wait for user input
      }
      while (Serial.read() != '\n');

      ultrasonicThreshold = Serial.parseInt();
      Serial.print("\nUltrasonic threshold set to: ");
      Serial.println(ultrasonicThreshold);

      // Read the ultrasonic sensor value
      //ultrasonicValue = readUltrasonicSensor();
      //logSensorData(ultrasonicValue, ldrValue); // Log the sensor data

      // Check if the ultrasonic sensor is outside the threshold in Automatic Mode
      if (automaticMode) {
        ultrasonicValue = readUltrasonicSensor();
        if (ultrasonicValue > ultrasonicThreshold) {
          setRgbLedColor(255, 0, 0); // Set LED to red
          Serial.println("\nUltrasonic sensor alert: Value exceeds threshold!");
        }
      }
    break;

    case 3:
      // LDR Alert Threshold
      Serial.print("Enter LDR threshold value: ");
      while (!Serial.available()) {
        // Wait for user input
      }
      ldrThreshold = Serial.parseInt();
      Serial.print("LDR threshold set to: ");
      Serial.println(ldrThreshold);

      // Read the LDR sensor value
      //ldrValue = analogRead(ldrPin);
      //logSensorData(ultrasonicValue, ldrValue); // Log the sensor data

      // Check if the LDR sensor is outside the threshold in Automatic Mode
      if (automaticMode) {
        ldrValue = analogRead(ldrPin);
        if (ldrValue > ldrThreshold) {
          setRgbLedColor(255, 0, 0); // Set LED to red
          Serial.println("\nLDR sensor alert: Value exceeds threshold!");
        }
      }
    break;

    case 4:
      // Back to main menu
      break;

    default:
      Serial.println("Invalid choice in Sensor Settings menu. Please enter a number between 1 and 4.");
  }
}

void processResetLoggerDataMenu() {
  Serial.println("\nReset Logger Data Menu:");
  Serial.println("1. Reset Ultrasonic Sensor Data");
  Serial.println("2. Reset LDR Sensor Data");
  Serial.println("3. Reset Both Sensors Data");
  Serial.println("4. Back");

  int choice;
  Serial.print("Enter your choice (1-4): ");
  while (!Serial.available()) {
    // Wait for user input
  }
  choice = Serial.parseInt();
  while (Serial.read() != '\n');

  Serial.print("\nReceived choice in Reset Logger Data menu: ");
  Serial.println(choice);

  switch (choice) {
    case 1:
      // Reset Ultrasonic Sensor Data
      Serial.println("\nAre you sure you want to reset Ultrasonic Sensor data? (1. Yes / 2. No)");
      while (!Serial.available()) {
        // Wait for user input
      }
      int confirmChoice = Serial.parseInt();
      while (Serial.read() != '\n');

      if (confirmChoice == 1) {
        // Reset Ultrasonic Sensor Data
        ultrasonicValue = 0;
        Serial.println("Ultrasonic Sensor Data reset.");
      } else {
        Serial.println("Reset operation canceled.");
      }
      break;

    case 2:
      // Reset LDR Sensor Data
      Serial.println("Are you sure you want to reset LDR Sensor data? (1. Yes / 2. No)");
      while (!Serial.available()) {
        // Wait for user input
      }
      confirmChoice = Serial.parseInt();
      while (Serial.read() != '\n');

      if (confirmChoice == 1) {
        // Reset LDR Sensor Data
        ldrValue = 0;
        Serial.println("LDR Sensor Data reset.");
      } else {
        Serial.println("Reset operation canceled.");
      }
      break;

    case 3:
      // Reset Both Sensors Data
      Serial.println("Are you sure you want to reset data for both sensors? (1. Yes / 2. No)");
      while (!Serial.available()) {
        // Wait for user input
      }
      confirmChoice = Serial.parseInt();
      while (Serial.read() != '\n');

      if (confirmChoice == 1) {
        // Reset Both Sensors Data
        ultrasonicValue = 0;
        ldrValue = 0;
        Serial.println("Both Sensors Data reset.");
      } else {
        Serial.println("Reset operation canceled.");
      }
      break;

    case 4:
      // Back to main menu
      break;

    default:
      Serial.println("Invalid choice in Reset Logger Data menu. Please enter a number between 1 and 4.");
  }
}


void processSystemStatusMenu() {
  Serial.println("\nSystem Status Menu:");
  Serial.println("1. Current Sensor Readings");
  Serial.println("2. Current Sensor Settings");
  Serial.println("3. Display Logged Data");
  Serial.println("4. Back");

  int choice;
  Serial.print("\nEnter your choice (1-4): ");
  while (!Serial.available()) {
    // Wait for user input
  }
  choice = Serial.parseInt();
  while (Serial.read() != '\n');

  Serial.print("\nReceived choice in System Status menu: ");
  Serial.println(choice);

  switch (choice) {
    case 1:
      // Current Sensor Readings
      Serial.println("Press any key to exit.");
      while (!Serial.available()) {
        // Display current sensor readings
        ultrasonicValue = readUltrasonicSensor();
        ldrValue = analogRead(ldrPin);

        Serial.print("Ultrasonic Sensor Reading: ");
        Serial.println(ultrasonicValue);

        Serial.print("LDR Sensor Reading: ");
        Serial.println(ldrValue);

        // Delay based on the defined interval
        delay(interval * 1000);
      }
      // Consume any remaining characters in the serial buffer
      while (Serial.read() != -1);
      Serial.println("Exiting Current Sensor Readings.");
      break;

    case 2:
      // Current Sensor Settings
      Serial.println("Current Sensor Settings:");
      Serial.print("Sampling Interval: ");
      Serial.print(interval);
      Serial.println(" seconds");

      Serial.print("Ultrasonic Alert Threshold: ");
      Serial.println(ultrasonicThreshold);

      Serial.print("LDR Alert Threshold: ");
      Serial.println(ldrThreshold);
      break;

    case 3:
      // Display Logged Data
      Serial.println("\nDisplaying last 10 sensor readings for both sensors:");

      for (int i = 0; i < MAX_LOG_ENTRIES; ++i) {
        int index = (logIndex + i) % MAX_LOG_ENTRIES;

        Serial.print("Entry ");
        Serial.print(i + 1);
        Serial.print(": Ultrasonic = ");
        Serial.print(ultrasonicLog[index]);
        Serial.print(", LDR = ");
        Serial.println(ldrLog[index]);
      }
    break;

    case 4:
      // Back to main menu
      break;

    default:
      Serial.println("Invalid choice in System Status menu. Please enter a number between 1 and 4.");
  }
}


void processRgbLedControlMenu() {
  Serial.println("RGB LED Control Menu:");
  Serial.println("1. Manual Color Control");
  Serial.println("2. LED: Toggle Automatic ON/OFF");
  Serial.println("3. Back");

  int choice;
  Serial.print("Enter your choice (1-3): ");
  while (!Serial.available()) {
    // Wait for user input
  }
  choice = Serial.parseInt();
  while (Serial.read() != '\n');

  Serial.print("\nReceived choice in RGB menu: ");
  Serial.println(choice);

  switch (choice) {
    case 1:
      // Manual Color Control
      Serial.println("Enter RGB values (0-255) separated by commas (e.g., 255,0,0 for red): ");
      while (!Serial.available()) {
        // Wait for user input
      }
      int red = Serial.parseInt();
      while (Serial.read() != ','); 
      int green = Serial.parseInt();
      while (Serial.read() != ',');
      int blue = Serial.parseInt();

      // Print the RGB values for testing
      Serial.print("\nReceived RGB values: ");
      Serial.print(red);
      Serial.print(", ");
      Serial.print(green);
      Serial.print(", ");
      Serial.println(blue);

      setRgbLedColor(red, green, blue);
      break;

    case 2:
      // Toggle Automatic Mode
      Serial.print("\nAutomatic Mode is ");
      Serial.println(automaticMode ? "ON" : "OFF");

      // If automatic mode is ON, set LED color based on sensor thresholds
      if (automaticMode) {
        // Perform sensor readings and set LED color accordingly
        int ultrasonicValue = readUltrasonicSensor();
        int ldrValue = analogRead(ldrPin);

        if (ultrasonicValue > ultrasonicThreshold || ldrValue > ldrThreshold) {
          setRgbLedColor(255, 0, 0); // Set LED to red
        } else {
          setRgbLedColor(0, 255, 0); // Set LED to green
        }
      }
      break;

    case 3:
      // Back to main menu
      break;

    default:
      Serial.println("Invalid choice in RGB menu. Please enter a number between 1 and 3.");
  }
}



// Function to load settings from EEPROM
void loadSettings() {
  int address = 0; // Define addresses for storing settings in EEPROM
  EEPROM.get(address, interval);
  address += sizeof(interval);

  EEPROM.get(address, ultrasonicThreshold);
  address += sizeof(ultrasonicThreshold);

  EEPROM.get(address, ldrThreshold);
  address += sizeof(ldrThreshold);

  EEPROM.get(address, automaticMode);
}


// Function to read ultrasonic sensor
int readUltrasonicSensor() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) * 0.034 / 2;
}


// Function to set RGB LED color
void setRgbLedColor(int red, int green, int blue) {
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}

void logSensorData(int ultrasonicValue, int ldrValue) {
  // Log the sensor data
  ultrasonicLog[logIndex] = ultrasonicValue;
  ldrLog[logIndex] = ldrValue;

  // Increment the index, and loop back to the beginning if it exceeds the maximum
  logIndex = (logIndex + 1) % MAX_LOG_ENTRIES;
}