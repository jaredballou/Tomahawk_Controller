/*
 * Tomahawk Controller
 * (c) 2021, Jared Ballou <github@jballou.com>
 * Major inspiration and code taken from:
 * Select Fire Rapidstrike by Monty Choy (Suild) - https://github.com/Suild/select-fire-rapidstrike-kit/blob/master/v1.0/Software/select-fire-rapidstrike-firmware/select-fire-rapidstrike-firmware.ino
 * Narfduino firmware by Michael Ireland (Airzone) - https://github.com/airzone-sama/Narfduino_Brushed_Auto_Solenoid/blob/main/Narfduino_Brushed_Auto_Solenoid.ino
 *
 * This is a program which runs the components of a Nerf blaster
 * via Arduino. The hardware consists of:
 * - Tomahawk 60 blaster
 * - 12v 3.5cm solenoid (dart pusher)
 * - Flywheel motors
 * - Fire selector switch
 * - 0.96" Display LCD
 *
 *
 */

/*-----( Import needed libraries )-----*/
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <arduino-timer.h>
#include <JC_Button.h>
#include <CircularBuffer.h>
#include <NarfduinoBattery.h>
#include <NarfduinoBridge.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define PIN_IN_PUSHER 6
#define PIN_OUT_PUSHER 4
#define PIN_IN_FIREMODE 2

#define PIN_IN_FLYWHEEL 7
#define PIN_OUT_FLYWHEEL_RUN 5
#define PIN_OUT_FLYWHEEL_STOP 15

#define PIN_OUT_FEED 8

#define _NARFDUINO_BATTERY_CALFACTOR 0.0

// Initialise the library
NarfduinoBattery Battery = NarfduinoBattery();

// Fire mode control
int stateFireMode = HIGH;
int readingFireMode;
int lockedFireMode;
int previousFireMode = LOW;
unsigned long timeFireMode = 0;
const char* fireModeName[] = {"SAFE","SEMI","BURST","AUTO"};
int fireModeCurrent = 0;

// Burst functionality, count and number of times to fire remaining
int burstCount = 3;
int burstRemaining = -1;

// Magazine capacity
int magazineMax = 60;
int magazineCurrent = magazineMax;

// Rev trigger
int stateRev = HIGH;      // the current state of the output pin
int readingRev;           // the current reading from the input pin
int previousRev = LOW;    // the previous reading from the input pin
unsigned long timeRev = 0;         // the last time the output pin was toggled
// Delay and timer for flywheel spin-up
unsigned long delayRev = 1000;
unsigned long doneRev = 0;

// Main trigger
int statePusher = HIGH;      // the current state of the output pin
int readingPusher;           // the current reading from the input pin
int previousPusher = LOW;    // the previous reading from the input pin
unsigned long timePusher = 0;         // the last time the output pin was toggled

// Delay and timer for pusher
unsigned long delayPusher = 250;
unsigned long donePusher = 0;
unsigned long doneFeed = 0;
unsigned long delayFeed = 250;

unsigned long debounce = 200;   // the debounce time, increase if the output flickers

// Battery level
/*-----( Declare objects )-----*/
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
/*-----( Declare Variables )-----*/

// Icons and bitmaps
#define ICON_HEIGHT   16
#define ICON_WIDTH    16
static const unsigned char PROGMEM icon_bullet[] = {
  0x00,0x00,
  0x00,0x00,
  0x00,0x00,
  0x00,0x00,
  0x3f,0xe0,
  0x20,0x30,
  0x20,0x18,
  0x20,0x0c,
  0x20,0x0c,
  0x20,0x18,
  0x20,0x30,
  0x3f,0xe0,
  0x00,0x00,
  0x00,0x00,
  0x00,0x00,
  0x00,0x00};
static const unsigned char PROGMEM icon_battery[] = {
  0x00,0x00,
  0x00,0x00,
  0x00,0x00,
  0x00,0x00,
  0x7f,0xfc,
  0x40,0x04,
  0x40,0x04,
  0x40,0x06,
  0x40,0x06,
  0x40,0x04,
  0x40,0x04,
  0x7f,0xfc,
  0x00,0x00,
  0x00,0x00,
  0x00,0x00,
  0x00,0x00};

void initSerial();
void initPins();
void initDisplay();
void initBattery();

void setup()   /*----( SETUP: RUNS ONCE )----*/
{
  initSerial();
  initPins();
  initDisplay();
  initBattery();
}
/*--(end setup )---*/
void initSerial() {
  Serial.begin( 57600 );
}

void initPins() {
  pinMode(PIN_IN_FIREMODE, INPUT_PULLUP);
  //pinMode(inPinMagazine, INPUT);
  pinMode(PIN_IN_FLYWHEEL, INPUT_PULLUP);
  pinMode(PIN_IN_PUSHER, INPUT_PULLUP);

  pinMode(PIN_OUT_FLYWHEEL_RUN, OUTPUT);
  pinMode(PIN_OUT_PUSHER, OUTPUT);
}

void initDisplay() {
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  } else {
    display.display();
    display.clearDisplay();
  }

}

void initBattery() {
  // Initialise the library
  Battery.Init();
  // Detect the battery size here.
  Battery.SetupSelectBattery();
  // Alternatively use .SetBatteryS( x ); where x is the number of S in the battery

  // Display the result of the detection
  Serial.print( "Battery Connected S = " );
  Serial.println( Battery.GetBatteryS() );
}

// Draw functions for OLED screen
void drawFireModeSafe() {
  display.drawRect(0, 0, 64, 16, SSD1306_WHITE);
  display.drawLine(0, 15, 15, 0, SSD1306_WHITE);
  display.drawLine(0, 0, 15, 15, SSD1306_WHITE);
  display.drawBitmap(
    0,
    0,
    icon_bullet, ICON_WIDTH, ICON_HEIGHT, 1);
}
void drawFireModeSemi() {
  display.drawRect(0, 0, 64, 16, SSD1306_WHITE);
  display.drawBitmap(
    0,
    0,
    icon_bullet, ICON_WIDTH, ICON_HEIGHT, 1);
}
void drawFireModeBurst() {
  display.drawRect(0, 0, 64, 16, SSD1306_WHITE);
  int16_t i;
  for(i=0; i<3; i++) {
    display.drawBitmap(
      ICON_WIDTH*i,
      0,
      icon_bullet, ICON_WIDTH, ICON_HEIGHT, 1);
  }
}

void drawFireModeAuto() {
  display.drawLine(0, 0, 0, 15, SSD1306_WHITE);
  display.drawLine(0, 0, 64, 0, SSD1306_WHITE);
  display.drawLine(0, 15, 64, 15, SSD1306_WHITE);
  int16_t i;
  for(i=0; i<4; i++) {
    display.drawBitmap(
      ICON_WIDTH*i,
      0,
      icon_bullet, ICON_WIDTH, ICON_HEIGHT, 1);
  }
}
void drawFireMode() {
    switch (fireModeCurrent) {
    case 0: //SAFE
      drawFireModeSafe();
      break;
    case 1: //SEMI
      drawFireModeSemi();
      break;
    case 2: //BURST
      drawFireModeBurst();
      break;
    case 3: //AUTO
      drawFireModeAuto();
      break;
  }
}
void drawBatteryMeter() {
  int left = display.width() - ICON_WIDTH*2;
  /*
  display.drawBitmap(
      left,
      0,
      icon_battery, ICON_WIDTH, ICON_HEIGHT, 1);
      */
  // Battery body
  display.drawRect(left, 1, ((ICON_WIDTH*2)-4), 14, SSD1306_WHITE);
  // Battery tip
  display.drawRect((display.width()-4),6,2,4, SSD1306_WHITE);
  // Size to make "fill" representing capacity
  /*
  for (int i=0; i<=(Battery.GetBatteryPercent()/20); i++) {
    float pipWidth = 4;
    display.fillRect(left+2+((pipWidth+1)*i), 3, pipWidth, 10, SSD1306_WHITE);
  }
  */
  float fillWidth = ((ICON_WIDTH*2-8) * (float(Battery.GetBatteryPercent())/100));
  display.fillRect(left+2, 3, fillWidth, 10, SSD1306_WHITE);
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_INVERSE); // Draw white text
  display.setCursor(left+4, 5);
  display.print(Battery.GetBatteryPercent());
  //display.drawRect(left, 5, (12 * (Battery.GetBatteryPercent())), 12, SSD1306_WHITE);
}
void drawMagazine() {
  display.setTextSize(6);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(32, 24);
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.print(magazineCurrent);
  /*
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(80, 5);
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.print(magazineCurrent);
  display.print("/");
  display.print(magazineMax);
  */
}
void updateScreen() {
  display.clearDisplay();
  drawFireMode();
  drawBatteryMeter();
  drawMagazine();
  display.display();
}
/*
void oldShit() {

  //lcd.clear();
  //Fire Mode
  lcd.setCursor(0, 0);
  lcd.print(fireModeName[fireModeCurrent]);
  lcd.print("  ");
  lcd.print(burstRemaining);

  // Magazine
  lcd.setCursor(16, 0);
  lcd.print(magazineCurrent);
  lcd.print(" ");
  lcd.setCursor(0, 1);
  lcd.print("Current time: ");
  lcd.print(millis());
  // Feed Advance
  lcd.setCursor(0, 1);
  lcd.print("Feed Advance: ");
  //lcd.print(doneFeed);

  // Rev Trigger
  lcd.setCursor(0, 2);
  lcd.print("Rev ");
  if (stateRev)
    lcd.print("On ");
  else
    lcd.print("Off");
  lcd.print(" ");
  lcd.print(doneRev);

  // Fire Trigger
  lcd.setCursor(0, 3);
  lcd.print("Pusher ");
  if (statePusher)
    lcd.print("On ");
  else
    lcd.print("Off");
  lcd.print(" ");
  lcd.print(donePusher);
}
*/
// End draw functions

int updateFireMode(int newFireMode=-1) {
  if (newFireMode != -1)
    fireModeCurrent = newFireMode;

  readingFireMode = digitalRead(PIN_IN_FIREMODE);
  if (millis() - timeFireMode > debounce) {
    timeFireMode = millis();
    if (readingFireMode == HIGH) {
      stateFireMode = LOW;
      lockedFireMode=0;
    } else {
      if (!lockedFireMode) {
        if (fireModeCurrent == 3)
          fireModeCurrent = 0;
        else
          fireModeCurrent++;
        lockedFireMode=1;
      }
      stateFireMode = HIGH;
    }
  }
  previousFireMode = readingFireMode;
  return fireModeCurrent;
}

void updatePusher() {
  readingPusher = digitalRead(PIN_IN_PUSHER);
  if (millis() - timePusher > debounce) {
    if (readingPusher == HIGH) {
      statePusher = LOW;
      burstRemaining = -1;
    } else {
      statePusher = HIGH;
    }
    timePusher = millis();
  }
  //digitalWrite(PIN_OUT_PUSHER, statePusher);
  previousPusher = readingPusher;
}

void updateRev() {
  readingRev = digitalRead(PIN_IN_FLYWHEEL);
  if (millis() - timeRev > debounce) {
    if (readingRev == HIGH) {
      stateRev = LOW;
      doneRev = 0;
    } else {
      stateRev = HIGH;
    }
    timeRev = millis();
  }
  digitalWrite(PIN_OUT_FLYWHEEL_RUN, stateRev);
  previousRev = readingRev;
}

int updateMagazine(int newMagazine=-1) {
  if (newMagazine != -1)
    magazineCurrent = newMagazine;
  return magazineCurrent;
}

void updateBattery() {
  static unsigned long LastTimeDisplayed = 0;

  // Run this frequently
  Battery.ProcessBatteryMonitor();
  return;

  if( millis() - LastTimeDisplayed > 1000 )
  {
    LastTimeDisplayed = millis();

    //Show the Voltage
    Serial.print( "Battery Voltage = " );
    Serial.println( Battery.GetCurrentVoltage() );

    // Show the % left
    Serial.print( "Battery % = " );
    Serial.println( Battery.GetBatteryPercent() );

    // Is the battery flat yet?
    Serial.print( "Battery Flat = " );
    Serial.println( Battery.IsBatteryFlat() );
  }
}
void doFireDart() {
  donePusher = millis() + (delayPusher*2);
  digitalWrite(PIN_OUT_PUSHER, HIGH);
  delay(50);
  digitalWrite(PIN_OUT_PUSHER, LOW);
  delay(delayPusher);
}

// Advance feed mechanism
void doFeedAdvance() {
  delay(delayFeed);
  doneFeed = millis() + (delayFeed*4);
  digitalWrite(PIN_OUT_FEED, HIGH);
  delay(delayFeed*2);
  digitalWrite(PIN_OUT_FEED, LOW);
}

// Attempt to fire a dart
void checkFireDart() {
  switch (fireModeCurrent) {
    case 0: //SAFE
      return;
      break;
    case 1: //SEMI
      if (burstRemaining == -1)
        burstRemaining = 1;
      break;
    case 2: //BURST
      if (burstRemaining == -1)
        burstRemaining = burstCount;
      break;
    case 3: //AUTO
      burstRemaining = 1;
      break;
  }
  if (doneRev && (doneRev <= millis())) {
    if ((doneFeed <= millis()) && (donePusher <= millis())) {
      if (burstRemaining) {
        burstRemaining--;
        magazineCurrent--;
        updateScreen();
        doFireDart();
        doFeedAdvance();
      }
    }
  }
}

// Start flywheel motors
void checkRev() {
  if (!doneRev)
    doneRev = millis() + delayRev;
}

// Check conditions to see if we need to fire a dart
void checkFireControl() {
  if (!fireModeCurrent)
    return;
  if (stateRev) {
    checkRev();
  }
  if (statePusher && stateRev) {
    checkFireDart();
  }
}

// Main loop
void loop()
{
  updateRev();
  updatePusher();
  updateFireMode();
  updateMagazine();
  updateBattery();
  checkFireControl();
  updateScreen();
}
