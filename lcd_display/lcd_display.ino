#include <EEPROM.h>
#include <LiquidCrystal.h>

// LCD Display variables
const byte rs = 9;
const byte en = 8;
const byte d4 = 7;
const byte d5 = 6;
const byte d6 = 5;
const byte d7 = 4;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Joystick variables
const int joystickButtonPin = 2;
const int xPin = A0;
const int yPin = A1;

// Thresholds for detecting joystick movement
const int minThreshold = 200;
const int maxThreshold = 600;
bool joyMoved = false;

// Menu variables
int currentOption = 1;

const int totalOptions = 3;
const int optionsPerPage = 2;
String menuItems[] = {"Start Game", "Settings", "About"};
String submenuItems[] = {"LCD Brightness", "Matrix Brightness", "Back"};


// Menu control variables!!!!!!!!!!!!!!!!!!!!!
int menuPage = 0;
int maxMenuPages = round(((sizeof(menuItems) / sizeof(String)) / 2) + .5);
int cursorPosition = 0;
int submenuPage = 0;
int maxSubmenuPages = round(((sizeof(submenuItems) / sizeof(String)) / 2) + .5);


//LCD Brightness control variables
int lcdBrightness;
int eepromAddress_lcdBrightness = 0;  // EEPROM address for storing/loading LCD brightness


// OTHER VARIABLES
const int debounceDelay = 50; // Adjust this value as needed
unsigned long lastButtonPress = 0;
unsigned long introMessageStartTime;

// Creates 3 custom characters for the menu display
byte downArrow[8] = {
  0b00100, //   *
  0b00100, //   *
  0b00100, //   *
  0b00100, //   *
  0b00100, //   *
  0b10101, // * * *
  0b01110, //  ***
  0b00100  //   *
};

byte upArrow[8] = {
  0b00100, //   *
  0b01110, //  ***
  0b10101, // * * *
  0b00100, //   *
  0b00100, //   *
  0b00100, //   *
  0b00100, //   *
  0b00100  //   *
};

byte menuCursor[8] = {
  B01000, //  *
  B00100, //   *
  B00010, //    *
  B00001, //     *
  B00010, //    *
  B00100, //   *
  B01000, //  *
  B00000  //
};


void setup() {
  Serial.begin(9600);
  // Set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.clear();

  pinMode(joystickButtonPin, INPUT_PULLUP); // activate pull-up resistor
  
  introMessageStartTime = millis();

  // Creates the byte for the 3 custom characters
  lcd.createChar(0, menuCursor);
  lcd.createChar(1, upArrow);
  lcd.createChar(2, downArrow);

  EEPROM.get(eepromAddress_lcdBrightness, lcdBrightness);
}

void loop() {
  // Check if the intro message should still be displayed
  if (millis() - introMessageStartTime < 3000) {
    // Display intro message
  lcd.setCursor(0, 0);
  lcd.print("Welcome to Your");
  lcd.setCursor(0, 1);
  lcd.print("Project!");
  } else {
    // The intro message has been displayed for 3 seconds, proceed with the main loop
    mainMenuDraw();
    drawCursor();
    operateMainMenu();
  }
}


// This function will generate the 2 menu items that can fit on the screen. 
//They will change as you scroll through your menu. Up and down arrows will indicate your current menu position.
void mainMenuDraw() {
  Serial.print(menuPage);
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(menuItems[menuPage]);
  lcd.setCursor(1, 1);
  lcd.print(menuItems[menuPage + 1]);
  if (menuPage == 0) { // User can only scroll down
    lcd.setCursor(15, 1);
    lcd.write(byte(2));
  } else if (menuPage > 0 and menuPage < maxMenuPages) { // User can scroll up & down
    lcd.setCursor(15, 1);
    lcd.write(byte(2));
    lcd.setCursor(15, 0);
    lcd.write(byte(1));
  } else if (menuPage == maxMenuPages) { // User can only scroll up
    lcd.setCursor(15, 0);
    lcd.write(byte(1));
  }
}


// When called, this function will erase the current cursor and redraw it based on the cursorPosition and menuPage variables.
void drawCursor() {
  for (int i = 0; i < 2; i++) {     // Erases current cursor
    lcd.setCursor(0, i);
    lcd.print(" ");
  }

  // The menu is set up to be progressive (menuPage 0 = Item 1 & Item 2, menuPage 1 = Item 2 & Item 3, menuPage 2 = Item 3 & Item 4), so
  // in order to determine where the cursor should be you need to see if you are at an odd or even menu page and an odd or even cursor position.
  if (menuPage % 2 == 0) {
    if (cursorPosition % 2 == 0) {  // If the menu page is even and the cursor position is even that means the cursor should be on line 1
      lcd.setCursor(0, 0);
      lcd.write(byte(0));
    } else { // If the menu page is even and the cursor position is odd that means the cursor should be on line 2
        lcd.setCursor(0, 1);
        lcd.write(byte(0));
    }
  } else {
    if (cursorPosition % 2 == 0) {  // If the menu page is odd and the cursor position is even that means the cursor should be on line 2
      lcd.setCursor(0, 1);
      lcd.write(byte(0));
    } else {  // If the menu page is odd and the cursor position is odd that means the cursor should be on line 1
      lcd.setCursor(0, 0);
      lcd.write(byte(0));
    }
  }
}


void operateMainMenu() {
  int activeButton = 0;
  int prevJoystickDirection = 0;

  while (activeButton == 0){
    int joystickY = analogRead(yPin);
    int joystickDirection = evaluateJoystickDirection(joystickY);

    if(joystickDirection != prevJoystickDirection){
      switch (joystickDirection) {
        case 0:
          break;
        case 1:  // Joystick moved up
          if (cursorPosition > 0) {
            cursorPosition--;
          } else if (menuPage > 0) {
            menuPage--;
            cursorPosition = 1;  // Move to the second item on the new page
          }
          break;
        case 2:  // Joystick moved down
          if (cursorPosition < 1) {
            cursorPosition++;
          } else if (menuPage < maxMenuPages) {
            menuPage++;
            cursorPosition = 0;  // Move to the first item on the new page
          }
          break;
      }

      mainMenuDraw();
      drawCursor();
    }
    
    // Check if the button is pressed
    if (digitalRead(joystickButtonPin) == LOW && millis() - lastButtonPress > debounceDelay) {
      Serial.println("Button pressed!"); //TEST
      lastButtonPress = millis();
      handleMenuSelection(menuPage * optionsPerPage + cursorPosition); // Joystick button is pressed, select the current menu option
      //activeButton = 1;
      mainMenuDraw();
      drawCursor();
    }

    prevJoystickDirection = joystickDirection;
  }
}



int evaluateJoystickDirection(int y) {
  if (y < minThreshold) {
    return 1;  // Joystick moved up
  } else if (y > maxThreshold) {
    return 2;  // Joystick moved down
  }

  return 0;  // No movement
}



// Handle the selection logic based on the selected menu option
void handleMenuSelection(int selectedOption) {
  switch (selectedOption) {
    case 0:
      menuItem1();
      break;
    case 1:
      menuItem2();
      break;
    case 2:
      menuItem3();
      break;
  }
}


// START GAME
void menuItem1() {
  lcd.clear();
  lcd.print("Starting Game...");
  //delay(1000);
}



// START GAME
void menuItem2() {
  Serial.print(menuPage);
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(submenuItems[menuPage]);
  lcd.setCursor(1, 1);
  lcd.print(submenuItems[menuPage + 1]);

  menuPage = 0;
  cursorPosition = 0;
  drawCursor();

  int activeButton = 0;
  int prevJoystickDirection = 0;

  while (activeButton == 0){
    int joystickY = analogRead(yPin);
    int joystickDirection = evaluateJoystickDirection(joystickY);

    if(joystickDirection != prevJoystickDirection){
      switch (joystickDirection) {
        case 0:
          break;
        case 1:  // Joystick moved up
          if (cursorPosition > 0) {
            cursorPosition--;
          } else if (menuPage > 0) {
            menuPage--;
            cursorPosition = 1;  // Move to the second item on the new page
          }
          break;
        case 2:  // Joystick moved down
          if (cursorPosition < 1) {
            cursorPosition++;
          } else if (menuPage < maxMenuPages) {
            menuPage++;
            cursorPosition = 0;  // Move to the first item on the new page
          }
          break;
      }

      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print(submenuItems[menuPage]);
      lcd.setCursor(1, 1);
      lcd.print(submenuItems[menuPage + 1]);
      drawCursor();
    }
    
    // Check if the button is pressed
    if (digitalRead(joystickButtonPin) == LOW && millis() - lastButtonPress > debounceDelay) {
      Serial.println("Button pressed!"); //TEST
      lastButtonPress = millis();
      handleSubmenuSelection(menuPage * optionsPerPage + cursorPosition); // Joystick button is pressed, select the current menu option
      //activeButton = 1;
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print(submenuItems[menuPage]);
      lcd.setCursor(1, 1);
      lcd.print(submenuItems[menuPage + 1]);
      drawCursor();
    }

    prevJoystickDirection = joystickDirection;
  }
  
}


// ABOUT
void menuItem3(){
  int numRows = 4;
  String about[] = {"Game Name: Mini Matrix Game", "Author: Ruka Mirela", "GitHub: github.com/Mirela89"};
  lcd.clear();

  for (int i = 0; i < numRows - 1; i++) {
    lcd.setCursor(0, 0);
    lcd.print("About");
    lcd.setCursor(0, 1);
    lcd.print(about[i]);
    delay(1000);

    for (int positionCounter = 0; positionCounter < 13; positionCounter++) {
      lcd.scrollDisplayLeft();
      delay(150);
    }

    // If it's not the last line, delay and clear the line
    if (i < numRows - 2) {
      delay(1000);
      lcd.setCursor(0, i);
      lcd.print("                ");  // Clear the line
    }
    lcd.clear();
  }
  delay(1000);  // Add an extra delay after the last line for better readability
}


void handleSubmenuSelection(int selectedOption) {
  switch (selectedOption) {
    case 0:
      //lcd.clear();
      //delay(1000);
      break;
    case 1:
      int potValue = analogRead(A0);  // Read potentiometer value (0-1023)
      int lcdBrightness = map(potValue, 0, 1023, 0, 255);  // Map to LCD brightness range (0-255)

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("LCD Brightness:");
      lcd.setCursor(0, 1);
      lcd.print(lcdBrightness);

      EEPROM.put(eepromAddress_lcdBrightness, lcdBrightness);

      delay(1000);
      break;
    case 2:
      return;
  }
}