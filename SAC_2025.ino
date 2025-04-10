#include "arduino_secrets.h"

#include "thingProperties.h"
#include "RTClib.h"
#include "HardwareSerial.h"
#include "Wire.h"


// GPIO 25 PWM Trig
// GPIO 26 Button 3
// GPIO 27 Button 1
// GPIO 14 7 Seg RX
// GPIO 12 Button 2


// ----- Pin Definitions -----
const int button1 = 27;            // Button for cancelling alarm LED fade
const int button2 = 12;            // Button for toggling lamp mode
const int brightnessButton = 26;   // GPIO26 for display brightness control
const int pwmTrig = 25;            // Pin for PWM LED Fading signal

// ----- 7-Segment Display Constants -----
#define DIGIT1  0x7B             // Display digit 1 control byte
#define DIGIT2  0x7C             // Display digit 2 control byte
#define DIGIT3  0x7D             // Display digit 3 control byte
#define DIGIT4  0x7E             // Display digit 4 control byte
#define COLON 4                  // Colon segment position in byte

// ----- Display Communication -----
HardwareSerial Serial7Segment(2); // Use UART2 (TX on GPIO14)
long colonTimer = 0;             // Timer for colon blinking
boolean colonOn = false;         // Current colon state

// ----- Brightness Configuration -----
byte brightnessLevels[] = {0, 51, 102, 153, 204, 255}; // 6 brightness steps (0=off)
byte brightnessIndex = 0;         // Current brightness level index
int buttonState3 = 0;             // Current brightness button state
int lastButtonState3 = 0;         // Previous brightness button state
bool displayActive = true;        // Display power state flag

// ----- PWM Control Variables -----
bool pwmActive = false;         // Tracks if PWM is enabled (true = max, false = off)
int lastButton2State = HIGH;      // For button2 state change detection (using INPUT_PULLUP)

// ----- RTC Configuration -----
RTC_DS3231 rtc;                  // Real-time clock object
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// ----- Alarm LED Fade Control Variables -----
// Global flag to track whether the alarm LED fade is active.
bool alarmActive = false;

// Non-blocking fade state machine for alarm LED fade
enum FadeState { FADE_IDLE, FADE_INITIAL, FADE_MAIN, FADE_DONE };
FadeState fadeState = FADE_IDLE;
unsigned long nextFadeUpdate = 0;   // Next scheduled update time for fade
int fadeStep = 0;                   // Index for initial fade steps
int currentPWM = 0;                 // Current PWM value
int localFadeTime = 1;              // Fade time in minutes from cloud variable
unsigned long delayInitial = 0;     // Delay for each initial fade step
unsigned long delayMain = 0;        // Delay for each main fade step
const int initialSteps[5] = {1, 2, 3, 4, 5}; // Initial fade steps

// ----------------------------------------------------------------------
// Function: startAlarmFade()
// This function initializes the non-blocking LED fade using the fade time
// obtained from the cloud variable fadeTime via getBrightness().
// ----------------------------------------------------------------------
void startAlarmFade() {
  // Ensure button1 is released before starting fade
  while (digitalRead(button1) == LOW) {
    delay(10);
  }
  localFadeTime = int(fadeTime.getBrightness());
  Serial.print("Using fadeTime (minutes): ");
  Serial.println(localFadeTime);
  // Calculate delays based on provided formulas:
  // For initial steps: delayInitial = (fadeTime * 60000) / 50
  delayInitial = (unsigned long)(localFadeTime * 60000UL) / 50;
  // For main fade: delayMain = (fadeTime * 60000) / 306
  delayMain = (unsigned long)(localFadeTime * 60000UL) / 306;
  
  fadeStep = 0;
  currentPWM = 0;
  fadeState = FADE_INITIAL;
  nextFadeUpdate = millis() + delayInitial;
  alarmActive = true;
  Serial.println("Alarm fade started (non-blocking)");
}

// ----------------------------------------------------------------------
// Function: updateAlarmFade()
// This function is called in the main loop to update the LED fade in a non-blocking
// manner. It checks for cancellation via button1 and updates the PWM value according
// to the fade state machine.
// ----------------------------------------------------------------------
void updateAlarmFade() {
  // Check if button1 is pressed to cancel the fade
  if (digitalRead(button1) == LOW) {
    delay(50); // Debounce
    if (digitalRead(button1) == LOW) {
      // Wait until button release to prevent repeated cancellations
      while (digitalRead(button1) == LOW) {
        delay(10);
      }
      alarmActive = false;
      alarmActivate = false; // Update cloud variable if needed
      fadeState = FADE_IDLE;
      analogWrite(pwmTrig, 0);
      Serial.println("Alarm cancelled manually via button1 (non-blocking)");
      return;
    }
  }
  
  unsigned long currentTime = millis();
  if (currentTime >= nextFadeUpdate) {
    if (fadeState == FADE_INITIAL) {
      if (fadeStep < 5) {
        currentPWM = initialSteps[fadeStep];
        analogWrite(pwmTrig, currentPWM);
        Serial.print("Initial fade step ");
        Serial.print(fadeStep);
        Serial.print(": PWM set to ");
        Serial.println(currentPWM);
        fadeStep++;
        nextFadeUpdate = currentTime + delayInitial;
      }
      if (fadeStep >= 5) {
        // Transition to main fade
        fadeState = FADE_MAIN;
        currentPWM = 6; // Starting PWM for main fade
        analogWrite(pwmTrig, currentPWM);
        Serial.println("Switching to main fade");
        nextFadeUpdate = currentTime + delayMain;
      }
    } else if (fadeState == FADE_MAIN) {
      if (currentPWM < 255) {
        currentPWM++;
        analogWrite(pwmTrig, currentPWM);
        Serial.print("Main fade: PWM set to ");
        Serial.println(currentPWM);
        nextFadeUpdate = currentTime + delayMain;
      } else {
        fadeState = FADE_DONE;
        Serial.println("LED fully on (Alarm active)");
      }
    } else if (fadeState == FADE_DONE) {
      // LED is fully on; maintain state and allow cancellation
    }
  }
}

// ----------------------------------------------------------------------
// Arduino Cloud Callback Functions
// These functions are triggered when the corresponding cloud variable changes.
// ----------------------------------------------------------------------
void onRtcTemperatureChange()  {
  // Executed every time a new value is received from IoT Cloud.
}

void onFadeTimeChange()  {
  // Executed every time a new value is received from IoT Cloud.
  // The new fadeTime will be used the next time the alarm triggers.
}

void onDisplayToggleChange()  {
  // When cloud variable changes, update display state
  if (displayToggle) {
    // Turn on display with last brightness level (if it was off, use max brightness)
    if (brightnessLevels[brightnessIndex] == 0) {
      brightnessIndex = 5; // Default to max brightness if previously off
    }
    setBrightness(brightnessLevels[brightnessIndex]);
  } else {
    setBrightness(0); // Turn off display
  }
}

void onLedBrightnessChange()  {
  // This function is triggered by IoT Cloud changes.
  Serial.println(ledBrightness.getBrightness());
  if (ledBrightness.getSwitch() == false)
    analogWrite(pwmTrig, 0);
  else
    analogWrite(pwmTrig, ledBrightness.getBrightness() * int(2.5));
}

// ----------------------------------------------------------------------
// Callback: onAlarmActivateChange()
// This function is executed whenever a new value is received from the IoT Cloud
// for the cloud variable alarmActivate. When alarmActivate becomes true, the LED
// fade is started in a non-blocking manner. When it becomes false, the LED is turned off.
// ----------------------------------------------------------------------
void onAlarmActivateChange()  {
  Serial.println("onAlarmActivateChange triggered");
  if (alarmActivate) {
    if (!alarmActive) {  // Prevent multiple concurrent fade processes
      startAlarmFade();
    }
  } else {
    alarmActive = false;
    fadeState = FADE_IDLE;
    analogWrite(pwmTrig, 0);
    Serial.println("Alarm deactivated via cloud variable");
  }
}

void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(9600);
  delay(1500); 
  
  // RTC initialization
  if (!rtc.begin()) {
    Serial.println("RTC Module Not Found");
    Serial.flush();
    while (1) delay(10); // Halt on critical failure
  }

  // 7-Segment Display Setup
  Serial7Segment.begin(9600, SERIAL_8N1, -1, 14); // Initialize UART
  Serial7Segment.write('v');  // Reset display to defaults
  
  // Brightness Control Setup
  pinMode(brightnessButton, INPUT_PULLUP);
  setBrightness(brightnessLevels[0]); // Start with display off

  // PWM LED Setup
  pinMode(pwmTrig, OUTPUT);
  analogWrite(pwmTrig, 0);
  
  // Configure button2 for PWM toggle
  pinMode(button2, INPUT_PULLUP);
  
  // Configure button1 for alarm cancellation
  pinMode(button1, INPUT_PULLUP);

  // Defined in thingProperties.h
  initProperties();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
}

// ----------------------------------------------------------------------
// Standard Arduino Loop Function
// ----------------------------------------------------------------------
void loop() {
  ArduinoCloud.update();     // Maintain IoT Cloud connection
  
  // Handle PWM control via button2:
  int currentButton2State = digitalRead(button2);
  // Check for button press (transition from HIGH to LOW)
  if (currentButton2State == LOW && lastButton2State == HIGH) {
    pwmActive = !pwmActive; // Toggle the PWM state

    // Calculation: PWM value = 255 for full duty cycle (enabled) or 0 for off.
    if (pwmActive) {
      analogWrite(pwmTrig, 255); // Set PWM to maximum (100% duty cycle)
    } else {
      analogWrite(pwmTrig, 0);   // Disable PWM (0% duty cycle)
    }
    delay(50); // Simple debounce delay
  }
  lastButton2State = currentButton2State;
  
  // Update temperature reading
  float currentTemp = rtc.getTemperature();
  if (rtcTemperature != currentTemp) {
    rtcTemperature = currentTemp; // Update cloud variable
  }

  // Get current time
  DateTime now = rtc.now();
  int currentHour = now.hour();
  int currentMinute = now.minute();
  
  // Update display with current time
  displayTime(currentHour, currentMinute);

  // Handle colon blinking (only when display active)
  if (displayActive && millis() - colonTimer >= 1000) {
    colonTimer = millis();
    toggleColon();
  }

  // Brightness button handling
  buttonState3 = digitalRead(brightnessButton);
  if (buttonState3 == LOW && lastButtonState3 == HIGH) {
    adjustBrightness(); // Cycle to next brightness level
  }
  lastButtonState3 = buttonState3; // Save state for next loop
  
  // If alarm fade is active, update it in a non-blocking fashion
  if (fadeState != FADE_IDLE) {
    updateAlarmFade();
  }
  
  delay(10); // Main loop pacing
}

// ----------------------------------------------------------------------
// 7-Segment Display Functions
// ----------------------------------------------------------------------

// Display current time on 7-segment display
void displayTime(int hour, int minute) {
  if (!displayActive) return; // Skip updates when display is off
  
  // Break time into individual digits
  sendDigit(DIGIT1, hour / 10);    // Hour tens place
  sendDigit(DIGIT2, hour % 10);    // Hour ones place
  sendDigit(DIGIT3, minute / 10);  // Minute tens place
  sendDigit(DIGIT4, minute % 10);  // Minute ones place
}

// Send digit pattern to specific display position
void sendDigit(byte digit, int value) {
  // 7-segment patterns for 0-9 (common cathode)
  byte digitPatterns[10] = {
    0b0111111, // 0
    0b0000110, // 1
    0b1011011, // 2
    0b1001111, // 3
    0b1100110, // 4
    0b1101101, // 5
    0b1111101, // 6
    0b0000111, // 7
    0b1111111, // 8
    0b1101111  // 9
  };
  Serial7Segment.write(digit);         // Select digit position
  Serial7Segment.write(digitPatterns[value]); // Send segment pattern
}

// Toggle colon visibility
void toggleColon() {
  colonOn = !colonOn;
  Serial7Segment.write(0x77);         // Colon control command
  Serial7Segment.write(colonOn ? (1 << COLON) : 0); // Set/clear colon bit
}

// Set display brightness (0-255)
void setBrightness(byte level) {
  if (level == 0) {
    clearDisplay();    // Physically turn off all segments
    displayActive = false; // Disable display updates
  } else {
    if (!displayActive) { // Waking from off state
      displayActive = true;
      colonTimer = millis(); // Reset colon timer
    }
    Serial7Segment.write(0x7A); // Brightness command
    Serial7Segment.write(level); // Set brightness level
  }
}

// Cycle through brightness levels for 7 seg display
void adjustBrightness() {
  brightnessIndex = (brightnessIndex + 1) % 6; // 0-5 index cycle
  setBrightness(brightnessLevels[brightnessIndex]);
}

// Turn off all display segments
void clearDisplay() {
  // Clear all four digits
  Serial7Segment.write(DIGIT1);
  Serial7Segment.write(0x00);
  Serial7Segment.write(DIGIT2);
  Serial7Segment.write(0x00);
  Serial7Segment.write(DIGIT3);
  Serial7Segment.write(0x00);
  Serial7Segment.write(DIGIT4);
  Serial7Segment.write(0x00);
  
  // Clear colon
  Serial7Segment.write(0x77);
  Serial7Segment.write(0x00);
}
