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

// Begin Suild code
// variables for managing all of firing state
#define ON true
#define OFF false

// Macros for pusher drive state
#define PUSHER_MOTOR_FIRING_MECHANISM 0
#define SOLENOID_FIRING_MECHANISM 1

#define PUSHER_DRIVE_STATE_OFF 0
#define PUSHER_DRIVE_STATE_BRAKE 1
#define PUSHER_DRIVE_STATE_ON 2

// Feed state. READY means one in the chamber, NEEDED meand empty chamber, ON means the solenoid is currently firing.
#define FEED_STATE_READY 0
#define FEED_STATE_NEEDED 1
#define FEED_STATE_ON 2

// Macros for rate of fire. Make sure FULL_AUTO is always last!
#define SAFETY 0
#define SEMI_AUTO 1
#define BURST_FIRE 2
#define FULL_AUTO 3

#define BURST_FIRE_LENGTH 3

struct firingState {
	uint8_t currentPusherState = PUSHER_DRIVE_STATE_OFF;
	// should only be mutated in controlMotors(), but can be accessed anywhere
	uint8_t targetPusherState = PUSHER_DRIVE_STATE_OFF;
  bool changingPusherState = false;
	// Keep track if pusher is firing. I can't just check if the pusher is being
	// powered because sometimes, the pusher is firing but off (for example, a
	// solenoid returning to its retracted position)
  bool isPusherFiring = false;

	uint8_t currentFeedState = FEED_STATE_NEEDED;
	uint8_t targetFeedState = FEED_STATE_READY;
  bool changingFeedState = false;
	bool isFeedFiring = false;

	// Keep track of firing mechanism, solenoid or pusher. This is updated on
	// trigger pull by reading the jumper
	bool pusherMechanism = SOLENOID_FIRING_MECHANISM;

	uint8_t currentFireMode = FULL_AUTO;


	// Keeps track of how many darts to fire in semi- and burst-fire modes
	uint8_t dartsToFire = 0;

	// Keep track of how many darts have been fired
	uint8_t dartsFired = 0;

	// Rate of fire from 0 to 100 (100 fastest)
	uint8_t rateOfFire = 100;

	// Off time between shots when in pusher motor mode. Higher rate of fire means
	// higher off time between shots. Units in ms
	uint8_t pusherMotorOffTime = 0;

	// Duty cyle for solenoid firing. Different fire rates will be a factor of
	// this. Units are percent
	const uint8_t SOLENOID_DUTY_CYCLE = 60;

	// Time that solenoid is powered when firing, in ms
	uint16_t solenoidOnTime = 35;

	// Time that solenoid is off when firing, in ms
	uint16_t solenoidOffTime = 15;

	// Timer to keep track of when to turn on/off the pusher
	Timer<> firingTimer = timer_create_default();

  Button btnPusher = Button(PIN_IN_PUSHER, 50, true, true);
  Button btnFlywheel = Button(PIN_IN_FLYWHEEL, 50, true, true);
  Button btnFireMode = Button(PIN_IN_FIREMODE, 50, true, true);

} firingState;

void LogMessage(const char* s) {
  Serial.println(s);
}


void updateSolenoidTiming();

void cycleFeedOn();
bool cycleFeedOff(void *);
void cycleFeedBegin();
bool cycleFeedComplete(void *);
bool handleFeedTransition(void *);

void cyclePusherOn();
bool cyclePusherOff(void *);
void cyclePusherBegin();
bool cyclePusherComplete(void *);
bool handlePusherTransition(void *);

void handlePusherSemiAutoAndBurst();
void handlePusherFullAuto();

void handleFiring();
void controlMotors();
void setRateOfFire();
void setDartsToFire();


void controlMotors() {
  //LogMessage("controlMotors");
	// If too much current, cut off power to load and don't run anything else
/*
  if (currentSenseState.isTooMuchCurrent) {
    if (millis() % 5000 == 0)
  		Serial.println("OCP");

		turnOffAllFETs();

		firingState.targetPusherState = PUSHER_DRIVE_STATE_OFF;
    firingState.isPusherFiring = false;

		return;
	}
  */
  if ((firingState.currentFeedState != FEED_STATE_READY) || (firingState.currentFeedState != firingState.targetFeedState)) {
    if (firingState.currentFeedState == FEED_STATE_NEEDED && !firingState.changingFeedState) {
      firingState.changingFeedState = true;
      firingState.targetFeedState = FEED_STATE_ON;
      firingState.firingTimer.in(1, handleFeedTransition);
    }
  }	else if (firingState.targetPusherState == PUSHER_DRIVE_STATE_OFF) {
    //turnOffAllFETs();

    firingState.currentPusherState = PUSHER_DRIVE_STATE_OFF;

  // Targetstate requires MOSFETs to change state that's at risk of shoot-
  // through
  // When transition FETs where the FETs are complementary, here's the sequence:
	// BRAKE -> OFF -> [shoot-through delay] -> ON
	// (or)
	// ON -> OFF -> [shoot-through delay] -> BRAKE
  					// Nake sure pusher state changed
  } else if (firingState.currentPusherState != firingState.targetPusherState
    && !firingState.changingPusherState
  	// Make sure target pusher state results in a complementary FET state
  	&& (firingState.targetPusherState == PUSHER_DRIVE_STATE_ON
  	|| firingState.targetPusherState == PUSHER_DRIVE_STATE_BRAKE)) {
      firingState.changingPusherState = true;
   	//turnOffAllFETs();

   	// Set flag to indicate that transition of FETs to a complementary state has
   	// begun
   	//fetState.hasComplementaryTransitionBegun = true;

   	// Transition FETs to match their target state afte SHOOT_THROUGH_DELAY
     firingState.firingTimer
   		.in(1, handlePusherTransition);
  }
}
bool handleFeedTransition(void *) {
  LogMessage("handleFeedTransition");
	if (firingState.targetFeedState == FEED_STATE_NEEDED) {
		digitalWrite(PIN_OUT_PUSHER, HIGH);
	} else if (firingState.targetPusherState == PUSHER_DRIVE_STATE_BRAKE) {
		digitalWrite(PIN_OUT_PUSHER, LOW);
	}

	// Reset flag, complementary transition complete
 	firingState.changingFeedState = false;

 	// Target pusher state has been achieved
	firingState.currentFeedState = firingState.targetFeedState;

	return true;		//required by timer lib
}

bool handlePusherTransition(void *) {
  LogMessage("handlePusherTransition");
	if (firingState.targetPusherState == PUSHER_DRIVE_STATE_ON) {
		digitalWrite(PIN_OUT_PUSHER, HIGH);
	} else if (firingState.targetPusherState == PUSHER_DRIVE_STATE_BRAKE) {
		digitalWrite(PIN_OUT_PUSHER, LOW);
	}

	// Reset flag, complementary transition complete
 	firingState.changingPusherState = false;

 	// Target pusher state has been achieved
	firingState.currentPusherState = firingState.targetPusherState;

	return true;		//required by timer lib
}

void handleFiring() {
  //LogMessage("handleFiring");
  if (firingState.currentFeedState == FEED_STATE_READY) {
  	if (firingState.btnPusher.wasPressed()) {
      setRateOfFire();
      setDartsToFire();
      firingState.isPusherFiring = true;
    }
    if (firingState.isPusherFiring) {
    	cyclePusherBegin();
    }
  } else {
    if (!firingState.isPusherFiring)
      cycleFeedBegin();
  }
}
void setRateOfFire() {
  LogMessage("setRateOfFire");
	uint8_t lowerLimitForRateOfFire = 1;
	uint8_t upperLimitForRateOfFire = 100;

	if (firingState.pusherMechanism == SOLENOID_FIRING_MECHANISM) {
		lowerLimitForRateOfFire = 20;
		upperLimitForRateOfFire = 35;
	}
  long MULTIPLIER = 1024;
	firingState.rateOfFire = map(
		MULTIPLIER, 0, 1024,
		upperLimitForRateOfFire, lowerLimitForRateOfFire);

	Serial.println(firingState.rateOfFire);
}

void setDartsToFire() {
  LogMessage("setDartsToFire");
  if (firingState.currentFireMode == SEMI_AUTO) {
    firingState.dartsToFire = 1;
  } else if (firingState.currentFireMode == BURST_FIRE) {
    firingState.dartsToFire = BURST_FIRE_LENGTH;
  }

  firingState.dartsFired = 0;
}
void cyclePusherBegin() {
  LogMessage("cyclePusherBegin");
	// Right when trigger was pulled, initiate firing sequence
  if (firingState.btnPusher.wasPressed()) {
		// Set solenoid on and off times based on rate of fire
		updateSolenoidTiming();

		// Turn solenoid on, set it to turn off later. Solenoid shouldn't be
		// turned on in safety
		if (firingState.currentFireMode != SAFETY) {
			firingState.targetPusherState = PUSHER_DRIVE_STATE_ON;
			firingState.firingTimer
				.in(firingState.solenoidOnTime, cyclePusherOff);

		// Firemode is safety
		} else {
			firingState.targetPusherState = PUSHER_DRIVE_STATE_BRAKE;
			firingState.isPusherFiring = false;
		}
	}

}

void cycleFeedBegin() {
  LogMessage("cycleFeedBegin");
	// Right when trigger was pulled, initiate firing sequence
  if (firingState.currentFeedState != FEED_STATE_READY) {
		// Set solenoid on and off times based on rate of fire
		updateSolenoidTiming();

		// Turn solenoid on, set it to turn off later. Solenoid shouldn't be
		// turned on in safety
		firingState.targetFeedState = FEED_STATE_ON;
		firingState.firingTimer
				.in(firingState.solenoidOnTime, cycleFeedOff);
	}

}
// Turns solenoid off and continues firing loop. Don't call this if you just
// want to turn off the solenoid
bool cycleFeedOff(void *) {
  LogMessage("cycleFeedOff");
	// Serial.println("Off");
	firingState.targetFeedState = FEED_STATE_READY;

	firingState.firingTimer
		.in(firingState.solenoidOffTime, cycleFeedComplete);
  return true;
}
bool cycleFeedComplete(void *) {
  LogMessage("cycleFeedComplete");
  if (firingState.btnPusher.isPressed()) {
    if (firingState.currentFireMode == SEMI_AUTO
      || firingState.currentFireMode == BURST_FIRE) {
      handlePusherSemiAutoAndBurst();
    } else if (firingState.currentFireMode == FULL_AUTO) {
      handlePusherFullAuto();
    } else {
      firingState.targetPusherState = PUSHER_DRIVE_STATE_BRAKE;
    }
  }
  return true;

}

// Sets solenoid timing based on rate of fire and duty cycle
void updateSolenoidTiming() {
  LogMessage("updateSolenoidTiming");
	//setRateOfFire();

	firingState.solenoidOnTime = (100.0 / firingState.rateOfFire)
		* (firingState.SOLENOID_DUTY_CYCLE / 2.0);

	firingState.solenoidOffTime = firingState.solenoidOnTime
		/ (firingState.SOLENOID_DUTY_CYCLE/(100.0/2.0));

	// Serial.print("On time "); Serial.println(firingState.solenoidOnTime);
	// Serial.print("Off time "); Serial.println(firingState.solenoidOffTime);
}

void handlePusherSemiAutoAndBurst() {
  LogMessage("handlePusherSemiAutoAndBurst");
  // If no more darts to fire, stop solenoid
  if (firingState.dartsFired >= firingState.dartsToFire) {
    firingState.targetPusherState = PUSHER_DRIVE_STATE_BRAKE;
    firingState.dartsFired = 0;
    firingState.isPusherFiring = false;

  // Still more darts to fire, so keep firing pusher and continue firing loop
  } else {
  	cyclePusherOn();
  }
}

void handlePusherFullAuto() {
  LogMessage("handlePusherFullAuto");
	// Trigger is let go, so turn off solenoid
  if (!firingState.btnPusher.isPressed()) {
    firingState.targetPusherState = PUSHER_DRIVE_STATE_BRAKE;
    firingState.isPusherFiring = false;

  // Trigger still pressed, so keep firing pusher and continue firing loop
  } else {
  	cyclePusherOn();
  }
}

// Turns solenoid off and continues firing loop. Don't call this if you just
// want to turn off the solenoid
bool cyclePusherOff(void *) {
  LogMessage("cyclePusherOff");
	// Serial.println("Off");
	firingState.targetPusherState = PUSHER_DRIVE_STATE_BRAKE;

	firingState.firingTimer
		.in(firingState.solenoidOffTime, cyclePusherComplete);
  return true;
}

// Called after solenoid turns off and about to turn back on. Ideally, this is
// executed when plunger in retraced position. Determines whether to continue
// firing the solenoid or to turn the solenoid off
bool cyclePusherComplete(void *) {
  LogMessage("cyclePusherComplete");
	firingState.dartsFired++;
  firingState.currentFeedState = FEED_STATE_NEEDED;
  firingState.targetFeedState = FEED_STATE_READY;
	// Update timing in case rate of fire changed
	updateSolenoidTiming();

  return true;
}

// Turns solenoid on and continues firing loop. Don't call this if you just
// want to turn on the solenoid
void cyclePusherOn() {
  LogMessage("cyclePusherOn");
	// Serial.println("On");
	firingState.targetPusherState = PUSHER_DRIVE_STATE_ON;

	firingState.firingTimer
 		.in(firingState.solenoidOffTime, cyclePusherOff);
}

void initButtons() {
  firingState.btnPusher.begin();
  firingState.btnFlywheel.begin();
  firingState.btnFireMode.begin();
}
void updateButtons() {
  firingState.btnPusher.read();
  firingState.btnFlywheel.read();
  firingState.btnFireMode.read();
}
// End suild stuff
// Begin Code
void setup()   /*----( SETUP: RUNS ONCE )----*/
{
  initSerial();
  initPins();
  initButtons();
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
    switch (firingState.currentFireMode) {
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
  lcd.print(fireModeName[firingState.currentFireMode]);
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
    firingState.currentFireMode = newFireMode;

  readingFireMode = digitalRead(PIN_IN_FIREMODE);
  if (millis() - timeFireMode > debounce) {
    timeFireMode = millis();
    if (readingFireMode == HIGH) {
      stateFireMode = LOW;
      lockedFireMode=0;
    } else {
      if (!lockedFireMode) {
        if (firingState.currentFireMode == 3)
          firingState.currentFireMode = 0;
        else
          firingState.currentFireMode++;
        lockedFireMode=1;
      }
      stateFireMode = HIGH;
    }
  }
  previousFireMode = readingFireMode;
  return firingState.currentFireMode;
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
  switch (firingState.currentFireMode) {
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
  if (!firingState.currentFireMode)
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
  firingState.firingTimer.tick();
  updateButtons();
  handleFiring();
 	controlMotors();

  updateRev();
  updatePusher();
  updateFireMode();
  updateMagazine();
  updateBattery();
  checkFireControl();
  updateScreen();
}
