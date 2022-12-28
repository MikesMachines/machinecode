
/*

MilkItCombo is a combination milker / fuck machine. This code was not meant to be released, it was 
mashed together from other machines. Many of the virtual pins are not needed - code for only a Milker 
will be significantly smaller.

There's a lot of workarounds for timing issues to run the stepper, update the app UI, a local screen,
and call Blynk.run() while running through iterations within a user defined cycle. That will be 
replaced.

*/

// This needs to be filled in (BLYNK needs to be at the top of this file)
#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID " "
#define BLYNK_DEVICE_NAME " "
#define BLYNK_AUTH_TOKEN " "
#define BLYNK_FIRMWARE_VERSION "0.14.04"

char ssid[] = " ";
char pass[] = " ";

// Stepper / or brushed motor (no longer used, this file is for a stepper)
#define MOTOR_TYPE_STEPPER
// #define MOTOR_TYPE_BRUSHED


#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Update.h>
#include <HTTPClient.h>
#include <loopTimer.h>
#include <ESP_FlexyStepper.h>


// This display is the UI of the blynk app
BlynkTimer timerUpdateDisplay;
WidgetTerminal terminal(V12);

// MOTOR Code - This file is for steppers
bool isMotorRunning = false;

// Stepper setup (used with tb660, DM542, DM556)
ESP_FlexyStepper stepper;
const int STEPS_PER_MM = 10;
const int MOTOR_STEP_PIN = 12;
const int MOTOR_DIRECTION_PIN = 27;

// Speed, acceleration and deceleration impact all operations. The strokelength is only used during bouncing.
int strokeLengthInSteps = 100;
int cycleStrokeLengthSteps = 100;
int speed_StepsPerSec = 200;
int acceleration_StepsPerSec = 1000;
int deceleration_StepsPerSec = 5000;
bool bounce = false;
bool runContinuous = false;

// Brushed DC - PWM Setup
// const int freq = 5000;
// const int pwmChannel = 0;
// const int PWMResolution = 8;
// const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) - 1);

// Brushed DC -Motor speed from 0 to 255. The min speed depends on the motor's minimum required voltage.
// In the UI speed gets normalized to the min / max as a factor of 0-100.
// int topSpeed = 255;
// int minSpeed = 100;
// int startingSpeed = 100;
// int currentSpeed = startingSpeed;
// int normalizedSpeed = 0;


// Config - esp32 ubs-c
// Suck and Blow is just a way to tie the solenoids to the UI, not very important
const int AIR_VALVE_PIN_BLOW = 14;
const int AIR_VALVE_PIN_SUCK = 26;



// Variables that can be set in the UI to change programs while running. For example,
// the airMultiplier can be used to increase (or decrease when < 1.0) the open time
// for a solenoid
float spaceMultiplier = 1.0;
float airMultiplier = 1.0;
float runMultiplier = 1.0;

// Trainwreck from combining too much old code. Iterations are the steps within
// a cycle. Nearly all code is work arounds to deal with Blynk timing used in
// the previuos version of Blynk
int cycleNumber = 0;
bool cycleRunning = false;
int currentIteration = 0;
int sizeOfIterations = 10;
long iterations[10];

// The cycles depend on timing. We need to keep separate times for debouncing each switch
// as well we updating the stepper info. For example, running continuously is done by adding
// steps to the target destination, we don't want to do it too often for performance, but if
// too long, we might end up waiting for it to reach its destination.
long debounceTime = 80;
unsigned long previousMillis = 0;
unsigned long previousLimitMillis = 0;
unsigned long previousEmergencyMillis = 0;
unsigned long continuousMillis = 300;
unsigned long previousContinuousMillis = 0;

// This is being replaced by 'isIterationComplete'
unsigned long iterationMillis = 0;


// There are some operations that require the motor be homed, also there are operations
//  that end in an unknown position canceling any previous homeing
bool isHomed = false;
long currentPosition = 0;

// A limit switch should never be hit during normal operation. THe homing interupt is
// detached during homing. Variables modified in ISR must be volitile.
const int HOME_LIMIT_PIN = 4;
const int MAX_STROKE_LIMIT_PIN = 5;
volatile int homeLimitChanged = 0;
volatile int emergencyLimitChanged = 0;


bool debugTerminal = false;

// No need to debounce - always brake when hit
void emergencySwitchHandler() {
  emergencyLimitChanged = 1;
  stepper.emergencyStop(true);
}


void homeLimitHandler() {
  if ((millis() - previousLimitMillis) >= debounceTime) {
    homeLimitChanged = 1;
    stepper.emergencyStop(true);
    previousLimitMillis = millis();
  }
}


void setup() {
  Serial.begin(115200);
  delay(200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  stepper.setDirectionToHome(1);
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
  stepper.setStepsPerMillimeter(10);
  stepper.setSpeedInStepsPerSecond(speed_StepsPerSec);
  stepper.setAccelerationInStepsPerSecondPerSecond(acceleration_StepsPerSec);
  stepper.setDecelerationInStepsPerSecondPerSecond(deceleration_StepsPerSec);
  stepper.startAsService();
  stepper.emergencyStop(true);

  pinMode(AIR_VALVE_PIN_BLOW, OUTPUT);
  pinMode(AIR_VALVE_PIN_SUCK, OUTPUT);
  pinMode(HOME_LIMIT_PIN, INPUT_PULLUP);
  pinMode(MAX_STROKE_LIMIT_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(HOME_LIMIT_PIN), homeLimitHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(MAX_STROKE_LIMIT_PIN), emergencySwitchHandler, CHANGE);


  timerUpdateDisplay.setInterval(2000L, updateDisplay);

  // Start with brake on
  Blynk.virtualWrite(V0, 1);
  Blynk.virtualWrite(V5, speed_StepsPerSec);
  Blynk.virtualWrite(V6, acceleration_StepsPerSec);
  Blynk.virtualWrite(V7, deceleration_StepsPerSec);
  Blynk.virtualWrite(V8, strokeLengthInSteps);
  Blynk.virtualWrite(V10, bounce);
  Blynk.virtualWrite(V14, cycleNumber);
  Blynk.virtualWrite(V18, runContinuous);


  terminal.clear();
  terminal.println(F("MilkThrust OTA v" BLYNK_FIRMWARE_VERSION ": E32-BlynkDev04 started"));
  terminal.flush();
}


void loop() {
  loopTimer.check(Serial);

  if (emergencyLimitChanged != 0) {
    if (millis() - previousEmergencyMillis >= 3000) {
      Serial.printf("emergencyLimitChanged = %i\n", emergencyLimitChanged);
      previousEmergencyMillis = millis();
      emergencyLimitChanged = 0;
    }
  }

  if (homeLimitChanged != 0) {
    if ((millis() - previousLimitMillis) >= debounceTime) {
      Serial.printf("homeLimitValue = %i\n", homeLimitChanged);
      previousLimitMillis = millis();
      homeLimitChanged = 0;
    }
  }

  Blynk.run();
  // timerUpdateDisplay.run();

  // Needs to be a function of speed - setting staticly means as it goes faster it will exceed the target and stop momentarily
  if ((runContinuous) && ((millis() - previousContinuousMillis) >= continuousMillis)) {
    previousContinuousMillis = millis();
    stepper.setTargetPositionInSteps(stepper.getCurrentPositionInSteps() + 500);
  }

  // Should require homing
  // The reason for the non-zero check is to provide a buffer to ensure the motor isn't waiting for this to be called.
  else if (bounce) {
    if (stepper.getCurrentPositionInSteps() <= 1) {
      stepper.setTargetPositionInSteps(strokeLengthInSteps);
    } else if (stepper.getCurrentPositionInSteps() >= strokeLengthInSteps) {
      stepper.setTargetPositionInSteps(0);
      // stepper.setTargetPositionRelativeInSteps(strokeLengthInSteps * -1);
    }
  }

  // All patterns, iterations, cycle functionality has been removed to simplyify for the milker.
  else if (cycleRunning) {

    if ((millis() - previousMillis) >= iterationMillis) {
      if (currentIteration > sizeOfIterations) {
        currentIteration = 0;
      } else {
        currentIteration++;
      }
      // For cycles with no iterations, this should have 0 delay
      iterationMillis = iterations[currentIteration];
      previousMillis = millis();
      runPattern();
    }
  }
  // if (currentIteration > sizeOfIterations) {
  //   currentIteration = 0;
  // } else {
  //   currentIteration++;
  // }

  //     if (stepper.getCurrentPositionInSteps() <= 3) {
  //       stepper.setTargetPositionInSteps(cycleStrokeLengthSteps + 3);
  //       runPattern();
  //     } else if (stepper.getCurrentPositionInSteps() >= cycleStrokeLengthSteps) {
  //       stepper.setTargetPositionInSteps(0);
  //       runPattern();
  //     }
  //   }
  // }

  // // Run the next iteration of the cycle id needed
  // else if ((isMotorRunning) && (isHomed)) {
  //   if (cycleRunning) {
  //   if ((millis() - previousMillis) >= iterationMillis) {
  //     if (currentIteration > sizeOfIterations) {
  //       currentIteration = 0;
  //     } else {
  //       currentIteration++;
  //     }
  //     // For cycles with no iterations, this should have 0 delay
  //     iterationMillis = iterations[currentIteration];
  //     previousMillis = millis();
  //     runPattern();
  //   }
  // }
  // }
}

void outputPosition() {

  long currentPosition = stepper.getCurrentPositionInSteps();
  long targetPosition = stepper.getTargetPositionInSteps();
  long targetDistance = stepper.getDistanceToTargetSigned();

  String distStr = String(targetDistance);
  String tarStr = String(targetPosition);
  String curStr = String(currentPosition);
  Serial.printf("Current pos : %l\n", currentPosition);
  terminal.print(F("Current: "));
  terminal.print(curStr);
  terminal.print(F("  Dist: "));
  terminal.print(distStr);
  terminal.print(F("  Tar: "));
  terminal.println(tarStr);
  terminal.flush();
}



bool findHome() {

  Serial.println(F("Homing"));
  terminal.println("Homing started ...");

  bounce = false;
  isMotorRunning = false;
  cycleRunning = false;
  detachInterrupt(HOME_LIMIT_PIN);
  isHomed = stepper.moveToHomeInSteps(1, 10.0, 1500L, HOME_LIMIT_PIN);
  attachInterrupt(digitalPinToInterrupt(HOME_LIMIT_PIN), homeLimitHandler, RISING);

  // Add buffer so 0 is in front of the limit - the home limit switch should never be hit.
  if (isHomed) {
    stepper.moveToPositionInSteps(100);
    stepper.setCurrentPositionAsHomeAndStop();
  }

  terminal.print("Homing completed. Homed = ");
  terminal.println(isHomed);
  terminal.flush();

  return isHomed;
}



BLYNK_APP_CONNECTED() {
  terminal.println(F("Milk & Thrust v" BLYNK_FIRMWARE_VERSION ": Device started"));
  terminal.flush();
  // Blynk.syncVirtual(V5);
  // Blynk.syncAll();
}


BLYNK_WRITE(V0) {
  switch (param.asInt()) {
    case 0:
      Serial.println(F("Released emergency STOP"));
      terminal.println(F("Released emergency STOP"));
      stepper.releaseEmergencyStop();
      isMotorRunning = true;

      break;
    case 1:
      Serial.println(F("Emergency STOP engaged"));
      terminal.println(F("Emergency STOP engaged"));
      stepper.emergencyStop(true);
      stepper.setTargetPositionRelativeInSteps(0);
      currentPosition = stepper.getCurrentPositionInSteps();
      terminal.print(F("Current position : "));
      terminal.println(F(currentPosition));
      cycleRunning = false;
      isMotorRunning = false;
      bounce = false;
      runContinuous = false;
      break;
  }
  terminal.flush();
}

BLYNK_WRITE(V5) {
  speed_StepsPerSec = param.asFloat();
  stepper.setSpeedInStepsPerSecond(speed_StepsPerSec);
  Serial.printf("Requested stepsPersec : %f\n", speed_StepsPerSec);
  terminal.print(F("Requested stepsPersec "));
  terminal.println(speed_StepsPerSec);
  terminal.flush();
}


BLYNK_WRITE(V6) {
  int requestedAccel = param.asInt();
  Serial.print(F("Accel "));
  Serial.println(requestedAccel);
  terminal.print(F("Requested accel "));
  terminal.println(requestedAccel);
  terminal.flush();
  stepper.setAccelerationInStepsPerSecondPerSecond(requestedAccel);
}

BLYNK_WRITE(V7) {
  int requestedDecel = param.asInt();
  Serial.print(F("Decel "));
  Serial.println(requestedDecel);
  terminal.print(F("Requested Decel "));
  terminal.println(requestedDecel);
  terminal.flush();
  stepper.setDecelerationInStepsPerSecondPerSecond(requestedDecel);
}


BLYNK_WRITE(V9) {
  int directParam = param.asInt();
  switch (directParam) {
    case 0:
      Serial.println(F("ishomed false "));
      terminal.println(F("ishomed false "));
      isHomed = false;
      break;
    case 1:
      Serial.println(F("ishomed true "));
      terminal.println(F("ishomed true "));
      isHomed = true;
      break;
  }
  terminal.flush();
}

BLYNK_WRITE(V10) {
  switch (param.asInt()) {
    case 0:
      Serial.println(F("Not bouncing"));
      terminal.println(F("Not bouncing"));
      stepper.setTargetPositionRelativeInSteps(0);
      bounce = false;
      isMotorRunning = false;
      cycleRunning = false;
      runContinuous = false;
      break;
    case 1:
      Serial.println(F(" bouncing"));
      terminal.println(F("Bouncing"));
      bounce = true;
      isMotorRunning = true;
      runContinuous = false;
      cycleRunning = false;
      stepper.setTargetPositionInSteps(0);
      break;
  }
  currentPosition = stepper.getCurrentPositionInSteps();
  terminal.print(F("Current position : "));
  terminal.println(F(currentPosition));
  terminal.flush();
}


BLYNK_WRITE(V11) {
  strokeLengthInSteps = param.asInt();

  // TODO
  // Need to check current position
  if (bounce) {
    stepper.setTargetPositionInSteps(strokeLengthInSteps);
  }

  if (debugTerminal) {
    Serial.print(F("strokeLengthInSteps"));
    Serial.println(strokeLengthInSteps);
    terminal.print(F("strokeLengthInSteps "));
    terminal.println(strokeLengthInSteps);
    terminal.flush();
  }
}

BLYNK_WRITE(V12) {
  currentPosition = stepper.getCurrentPositionInSteps();
  terminal.print(F("Current position : "));
  terminal.println(F(currentPosition));
  terminal.flush();
}

// Not used by milker
BLYNK_WRITE(V14) {
  Serial.print(F("Requested cycle : "));
  Serial.println(param.asInt());
  cycleNumber = param.asInt();
  initIteration();
  cycleRunning = true;
  isMotorRunning = true;
}

BLYNK_WRITE(V15) {
  int moveTo = param.asInt();
  int negMoveTo = moveTo * -1;
  stepper.setTargetPositionInSteps(negMoveTo);
  cycleRunning = false;
  bounce = false;
  isMotorRunning = true;
  if (debugTerminal) {
    Serial.print(F("moveTo "));
    Serial.print(moveTo);
    Serial.print(F("   negmoveTo "));
    Serial.println(negMoveTo);

    terminal.print(F("moveTo "));
    terminal.print(moveTo);
    terminal.print(F("   negmoveTo "));
    terminal.println(negMoveTo);
    terminal.flush();
  }
}

BLYNK_WRITE(V16) {
  switch (param.asInt()) {
    case 1:
      Serial.print(F("Setting home  : "));
      Serial.println(stepper.getCurrentPositionInSteps());
      // findHome();
      cycleRunning = false;
      bounce = false;
      isMotorRunning = false;
      break;
  }
}

BLYNK_WRITE(V17) {
  outputPosition();
}

BLYNK_WRITE(V18) {
  switch (param.asInt()) {
    case 0:
      Serial.println(F("Stop continuous"));
      terminal.println(F("Stop continuous - now at position 0."));
      stepper.setCurrentPositionAsHomeAndStop();
      bounce = false;
      isMotorRunning = false;
      cycleRunning = false;
      runContinuous = false;
      isHomed = false;
      break;
    case 1:
      Serial.println(F("Starting continuous"));
      terminal.println(F("Starting continuous"));
      bounce = false;
      isMotorRunning = true;
      cycleRunning = false;
      isHomed = false;
      runContinuous = true;
      break;
  }

  terminal.flush();
}


BLYNK_WRITE(V20) {
  switch (param.asInt()) {
    case 0:
      Serial.println(F("airValve1Pin OFF"));
      digitalWrite(AIR_VALVE_PIN_BLOW, LOW);
      break;

    case 1:
      Serial.println(F("airValve1Pin ON"));
      digitalWrite(AIR_VALVE_PIN_BLOW, HIGH);
      break;
  }
}

BLYNK_WRITE(V21) {
  switch (param.asInt()) {
    case 0:
      Serial.println(F("airValve2Pin OFF"));
      digitalWrite(AIR_VALVE_PIN_SUCK, LOW);
      break;

    case 1:
      Serial.println(F("airValve2Pin ON"));
      digitalWrite(AIR_VALVE_PIN_SUCK, HIGH);
      break;
  }
}

BLYNK_WRITE(V22) {
  int openTime = 500 * airMultiplier;
  terminal.println(F("Open Air Valve 1"));
  terminal.flush();
  openAirValveFor(openTime);
}

BLYNK_WRITE(V23) {
  terminal.println(F("Open Air Valve 2"));
  terminal.flush();
  int openTime = 500 * airMultiplier;
  openAirValve2For(openTime);
}

BLYNK_WRITE(V30) {
  switch (param.asInt()) {
    case 0:
      Serial.println(F("Debug terminal off."));
      debugTerminal = false;
      break;

    case 1:
      Serial.println(F("Debug terminal ON"));
      debugTerminal = true;
      break;
  }
  terminal.print(F("Debug terminal = "));
  terminal.println(debugTerminal);
  terminal.flush();
}


BLYNK_WRITE(InternalPinOTA) {
  String overTheAirURL = param.asString();
  HTTPClient http;
  http.begin(overTheAirURL);
  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    return;
  }
  int contentLength = http.getSize();
  if (contentLength <= 0) {
    return;
  }
  bool canBegin = Update.begin(contentLength);
  if (!canBegin) {
    return;
  }
  Client& client = http.getStream();
  int written = Update.writeStream(client);
  if (written != contentLength) {
    return;
  }
  if (!Update.end()) {
    return;
  }
  if (!Update.isFinished()) {
    return;
  }
  reboot();
}

void reboot() {
#if defined(ARDUINO_ARCH_MEGAAVR)
  wdt_enable(WDT_PERIOD_8CLK_gc);
#elif defined(__AVR__)
  wdt_enable(WDTO_15MS);
#elif defined(__arm__)
  NVIC_SystemReset();
#elif defined(ESP8266) || defined(ESP32)
  ESP.restart();
#else
#error "MCU reset procedure not implemented"
#endif
  for (;;) {}
}


void openAirValveFor(float sec) {
  float delayTime = sec * airMultiplier;
  digitalWrite(AIR_VALVE_PIN_BLOW, HIGH);
  delay(sec);
  closeAirValve();
}

void openAirValveMultiplierFor(float sec) {
  float delayTime = sec * airMultiplier;
  openAirValveFor(delayTime);
}

void openAirValve2For(int sec) {
  digitalWrite(AIR_VALVE_PIN_SUCK, HIGH);
  delay(sec);
  closeAirValve2();
}

void openAirValve2MultiplierFor(float sec) {
  float delayTime = sec * airMultiplier;
  openAirValve2For(delayTime);
}

void closeAirValve() {
  digitalWrite(AIR_VALVE_PIN_BLOW, LOW);
}
void closeAirValve2() {
  digitalWrite(AIR_VALVE_PIN_SUCK, LOW);
}

// Used as a manual pause for calibration.
bool waitingForEmergencyLimitChanged() {
  int maxWaitTime = 30;
  while (emergencyLimitChanged == 0) {
    delay(500);
    Serial.printf("Waiting for emergency switch to continue.\n");
    maxWaitTime = maxWaitTime - 1;
    if (maxWaitTime < 1) {
      return false;
    }
  }

  return true;
}



// All user defined routines are stored in progmem. They can be modified via webclient upload.
// Iteration / patterns / cycle code here is just for testing purposes.
void runPattern() {

  switch (cycleNumber) {

    if (debugTerminal) {
      terminal.print(F("Cycle : "));
      terminal.print(cycleNumber);
      terminal.print(F("  Iteration:  "));
      terminal.println(currentIteration);
      terminal.flush();
    }

    case 1:
      if (currentIteration == 0) {
        stepper.setAccelerationInStepsPerSecondPerSecond(100);
        stepper.setSpeedInStepsPerSecond(200);
        cycleStrokeLengthSteps = 100;
        break;
      } else if (currentIteration == 1) {
        stepper.setSpeedInStepsPerSecond(500);
        Blynk.virtualWrite(V5, 500);
        stepper.setTargetPositionRelativeInSteps(1000);
        break;
      } else if (currentIteration == 2) {
        stepper.setSpeedInStepsPerSecond(200);
        Blynk.virtualWrite(V5, 200);
        stepper.setTargetPositionRelativeInSteps(500);
      }
      break;

    case 2:

      break;

    default:
      stepper.setSpeedInStepsPerSecond(speed_StepsPerSec);
      break;
  }
}


void initIteration() {

  switch (cycleNumber) {
    case 1:
      stepper.setAccelerationInStepsPerSecondPerSecond(1000);
      stepper.setDecelerationInStepsPerSecondPerSecond(1000);
      Blynk.virtualWrite(V6, 1000);
      Blynk.virtualWrite(V7, 1000);
      sizeOfIterations = 3;
      iterations[0] = (3500);
      iterations[1] = 9500;
      iterations[2] = 3000;
      break;

    case 2:
      sizeOfIterations = 3;
      iterations[0] = 900;
      iterations[1] = 900;
      iterations[2] = 400;
      break;

    case 3:
      sizeOfIterations = 1;
      iterations[0] = 1000;
      break;

    case 4:
      sizeOfIterations = 2;
      iterations[0] = 400;
      iterations[1] = 600;
      break;

    case 5:
      sizeOfIterations = 2;
      iterations[0] = (500 * runMultiplier);
      iterations[1] = (600 * spaceMultiplier);
      break;
  }
}

void updateDisplay() {
  // Blynk.virtualWrite(V5, speed_StepsPerSec);
  // Blynk.virtualWrite(V6, acceleration_StepsPerSec);
  // Blynk.virtualWrite(V7, deceleration_StepsPerSec);
  // Blynk.virtualWrite(V8, strokeLengthInSteps);
  // Blynk.virtualWrite(V14, cycleNumber);

  // Blynk.virtualWrite(V18, runContinuous);
  // Blynk.virtualWrite(V10, bounce);

  if (debugTerminal) {
    outputPosition();
  }
}

void runTest(int testNumber) {
  // stepper.releaseEmergencyStop();

  switch (testNumber) {
    case 1:
      stepper.moveToPositionInMillimeters(0);
      stepper.moveToPositionInMillimeters(100);
      stepper.moveToPositionInMillimeters(0);
      stepper.moveToPositionInMillimeters(200);
      stepper.moveToPositionInMillimeters(0);
      break;
    case 2:

      stepper.moveToPositionInMillimeters(0);
      stepper.moveToPositionInMillimeters(100);
      stepper.moveToPositionInMillimeters(0);
      stepper.moveToPositionInMillimeters(200);
      stepper.moveToPositionInMillimeters(0);
      break;
  }
  // while (!isHomed) {
  //   Serial.printf("Home not set... finding home.\n");
  //   delay(3000);
  //   findHome();
  // }
}
