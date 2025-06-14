// Code generated by Arduino IoT Cloud, DO NOT EDIT.

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>

const char DEVICE_LOGIN_NAME[]  = "4ed06da7-ad9e-43b5-b696-4b7b8419accb";

const char SSID[]               = SECRET_SSID;    // Network SSID (name)
const char PASS[]               = SECRET_OPTIONAL_PASS;    // Network password (use for WPA, or use as key for WEP)
const char DEVICE_KEY[]  = SECRET_DEVICE_KEY;    // Secret device password

void onFadeTimeChange();
void onLedBrightnessChange();
void onAlarmActivateChange();
void onDisplayToggleChange();

CloudDimmedLight fadeTime;
CloudDimmedLight ledBrightness;
CloudSwitch alarmActivate;
CloudSwitch displayToggle;
CloudTemperatureSensor rtcTemperature;

void initProperties(){

  ArduinoCloud.setBoardId(DEVICE_LOGIN_NAME);
  ArduinoCloud.setSecretDeviceKey(DEVICE_KEY);
  ArduinoCloud.addProperty(fadeTime, READWRITE, ON_CHANGE, onFadeTimeChange);
  ArduinoCloud.addProperty(ledBrightness, READWRITE, ON_CHANGE, onLedBrightnessChange);
  ArduinoCloud.addProperty(alarmActivate, READWRITE, ON_CHANGE, onAlarmActivateChange);
  ArduinoCloud.addProperty(displayToggle, READWRITE, ON_CHANGE, onDisplayToggleChange);
  ArduinoCloud.addProperty(rtcTemperature, READ, 10 * SECONDS, NULL);

}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);
