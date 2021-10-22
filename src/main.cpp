
#include <Arduino.h>
#include <Wire.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <vector>
#include <memory>
#include <ctime>
#include <string>
#include <NMEAGPS.h>
#include <SoftwareSerial.h>
#include <U8g2lib.h>
#include "mykeys.h" //header containing sensitive information. Not included in repo. You will need to define the missing constants.
#include "types.h"
#include "SharedUtils/Utils.h"
#include <SPIFFS.h>
//#include <esp_pm.h>

#define CONTROL_VOLTAGE  true //if true, PIN_BAT will be used to read the voltage of the battery
#define LOG_ACTIVE       false
#define FORCE_RESET_TIME (30*60*1000) //half an hour and no gps? reset!
#define SLEEP_TIME_SECS  120
#define MAX_SAME_LOCS    5
#define MAX_DIST_OK      5000        //if the distance with the previous gps point is greater than this, ignore the loc and do not publish.
#define SLEEPMODEM_NOUPT (6*60*1000) //No new GPS updates in 6 minutes? Sleep the modem!!
#define LSLEEP_WAIT      (20*1000)   //Enter light sleep after this time after publishing a result
#define LSLEEP_SECS      30          //Light sleep this time

#define MIN_DIST_UPDATE  12      //update gps if position has moved at least this meters
#define DIST_UPDATE_1    25      //if gps pos has moved this meters, update a little faster
#define DIST_UPDATE_2    50      //if gps pos has moved this meters, update faster
#define DIST_UPDATE_3    100     //if gps pos has moved this meters, update fastest
#define UPDATE_TIME      60000   //update once every this seconds if gps has moved between MIN_DIST_UPDATE and DIST_UPDATE_1
#define UPDATE_TIME_1    40000   //update once every this seconds if gps has moved between DIST_UPDATE_1 and DIST_UPDATE_2
#define UPDATE_TIME_2    30000   //update once every this seconds if gps has moved between DIST_UPDATE_2 and DIST_UPDATE_3
#define UPDATE_TIME_3    20000   //update once every this seconds if gps has moved more than DIST_UPDATE_3

#define CHECK_EVERY      (UPDATE_TIME_3/4) //check if GPS has moved once every this time
#define UPDATE_AT_LEAST  (20*60*1000)      //update at least once every 20 minutes.

#define MAX_VOLTAGE      4.32f //when charging
#define MIN_VOLTAGE      3.40f
#define MAX_VOLTAGE_READ 2225  //when charging

#define LED_BLINK_TIME   100
#define LED_BLINK_OFF    (LED_BLINK_TIME*15)
#define BLINKS_LOCATING  1
#define BLINKS_FIXED     2
#define BLINKS_GSMERROR  3

#define SIM800L_IP5306_VERSION_20200811
#include "SharedUtils/lilygo_utils.h"

#define PIN_GPS_TX   18
#define PIN_GPS_RX   19
#define PIN_I2C_SDA  21
#define PIN_I2C_SCL  22
#define PIN_ENABLE   12
#define PIN_BAT      35 //15
#define PIN_LED      2
#define PIN_GO_SLEEP 15

#define SCREEN_WIDTH   128 // OLED display width, in pixels
#define SCREEN_HEIGHT  64 // OLED display height, in pixels
#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for micro seconds to seconds

#define TRACKER_ID       1 //this is the 1st tracker!
#define MQTT_BROKER      "mitotoro.synology.me"
#define MQTT_PORT        1888
#define FEED_BABYTRACKER "babytracker/loc" //ID,datetime,latitude,longitude,elevation,speed
#define FEED_BATTERY     "babytracker/bat" //ID,datetime,latitude,longitude,elevation,speed

//U8G2_SH1106_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, PIN_I2C_SCL, PIN_I2C_SDA);
U8G2_SSD1306_128X64_NONAME_1_HW_I2C _u8g2(U8G2_R0, PIN_I2C_SCL, PIN_I2C_SDA);

#define MAX_GPRS_ERRORS        5  //after this errors, restart everything
#define FIXED_LOOPS_B4_PUBLISH 15 //once we have a fix, wait this loops before trying to publish the value

#define TINY_GSM_MODEM_SIM800 // Modem difinition... we are using this one (SIM800L)
#define TINY_GSM_DEBUG        Serial
#include <TinyGsmClient.h>


SoftwareSerial  _SerialGPS(PIN_GPS_RX, PIN_GPS_TX); // no more hw serials available. We have to use a software serial.
TinyGsm         _modemGSM(Serial1); //With LILYGO TCALL the modem rx/tx is this one
TinyGsmClient   _clientGSM(_modemGSM);

//WiFiClient      _TheWifi;
PubSubClient    _ThePubSub;

ScreenInfo      _TheScreenInfo;
bool            _ScreenActive = false;

NMEAGPS         _TheNeoGps;           // This object parses the GPS characters
gps_fix         _TheFix;              // This global structure contains the GPS fix returned by _TheNeoGps
GpsPosLocation  _LastPublishedPos;    // Last published coordinates
unsigned long   _LastPublishTime = 0; // last time we successfully published a location
unsigned long   _LastUpdCheck = 0;    // Last time we checked if an update was needed.
uint8_t         _CountSameLoc=0;      // To count number of times the location was exactly the same... if that happens is probably a bug --> reset

//TIMING VARS
unsigned long  _lastProcessMillis = 0;
unsigned long  _LastFixed = 0;
unsigned long  _lastLedChange = 0;
unsigned long  _lastModemON = 0;      // Last time we turned on the modem
unsigned long  _lastLightSleep = 0;   // Last time we entered light sleep
uint8_t        _GprsErrorCount = 0;
int            _timeout = 0;
uint16_t       _FixedLoops = 0;
bool           _ModemInitialized = false;
bool           _ScreenInitialized = false;
bool           _LedON = false;
uint8_t        _LedBlinks=0; //Counts the number of led blinks since the last _LedBlinks=0
uint16_t       _LastVoltageRead = MAX_VOLTAGE_READ;
unsigned long  _lastVoltageUpd = 0;
unsigned long  _AccumVoltage=0;
uint16_t       _NumVoltageReadings=0;
bool           _LocationPublished=false; //true if the gps location has been published at least once
bool           _LogAvailable=false;
File           _TheLog;

//FORWARD DECLARATIONS
void ReadFromSerial();
void DrawScreen();
void setupModem();
bool setupGSM();
bool verifyGPRSConnection();
bool verifyMqttConnection();
void RestartNOW();
void GoDeepSleep();
void SleepModem();
void AwakeModem();
bool EnableModem();
void LedControl();
void TurnModemNetlight(bool on);
bool UpdateNeeded(unsigned long now, float &dist);
uint16_t ReadAvgValue(uint8_t pin);
void LogMsg(std::string msg, bool add2log=true);
//END FF DECLARATIONS

void setup()
{
	Serial.begin(115200);
	// wait for serial monitor to open
	while(!Serial);

	pinMode(PIN_GO_SLEEP, INPUT_PULLUP); //If LOW, go deep sleep

	LogMsg(Utils::string_format("Current speed=[%dMHz]. Setting new speed...", getCpuFrequencyMhz()));
	if(setCpuFrequencyMhz(40)) LogMsg("OK!");
	else LogMsg("Failed!");
	LogMsg(Utils::string_format("Current speed=[%dMHz].", getCpuFrequencyMhz()));

	// Launch SPIFFS file system
	if(LOG_ACTIVE) {
		if(!SPIFFS.begin()) {
			LogMsg("An Error has occurred while mounting SPIFFS. Trying format...");
			if(SPIFFS.format()) {
				LogMsg("SPIFFS formated OK");//next reset will be able to write to log....
			}
		}
		else {
			_TheLog = SPIFFS.open("/log.txt", "a+");
			if(!_TheLog) {
				LogMsg("Failed to open LOG file :(");
			}
			else {
				_LogAvailable = true;
			}
			unsigned int totalBytes = SPIFFS.totalBytes();
			unsigned int usedBytes = SPIFFS.usedBytes();
			LogMsg(Utils::string_format("SPIFFS totalSize=[%d] usedSize=[%d]", totalBytes, usedBytes));
		}
	}

	if(digitalRead(PIN_GO_SLEEP)==LOW) {
		LogMsg("LOW --> GoDeepSleep", false);
		pinMode(LED_GPIO, OUTPUT);
		digitalWrite(LED_GPIO, LED_ON);
		delay(5000);
		GoDeepSleep();
	}
	else {
		LogMsg("HIGH --> WORK, my slave!!!", false);
	}

	// Start power management
	if(setupPMU() == false) {
		LogMsg(Utils::string_format("Setting power error"));
	}

	LogMsg(Utils::string_format("Enabling transistor..."));
	pinMode(PIN_ENABLE, OUTPUT); //We have the gps and the screen as the load of a transistor, so they fully disconnect when esp32 goes deep sleep
	digitalWrite(PIN_ENABLE, HIGH);
	pinMode(PIN_LED, OUTPUT);
	digitalWrite(PIN_LED, LOW);

	LogMsg(Utils::string_format("Setup Serial GPS..."));
	_SerialGPS.begin(9600);

	//We will setup the modem only when we have a gps fix, so we can safe battery

	// Start display
	LogMsg(Utils::string_format("Begin Display..."));
	if(_ScreenActive && _u8g2.begin()) {
		_ScreenInitialized=true;
		_u8g2.setFont(u8g2_font_5x8_mf);
		_TheScreenInfo.wifiState = "Sleeping...";
		_TheScreenInfo.mqttState = "Waiting GSM...";
		DrawScreen();
	}
	else {
		LogMsg(Utils::string_format("Screen NOT available"));
	}

	LogMsg(Utils::string_format("Setup Complete!"));
}

void loop()
{
	//Process any instrucction received from Serial
	ReadFromSerial();

	ReadAvgValue(PIN_BAT);
	//LogMsg(Utils::string_format("Voltage value=%d", (int)(voltageRead));

	float dist=0;
	auto now = millis();
	if(_TheScreenInfo.gpsFix && (now - _LastFixed) > 60000) {
		_TheScreenInfo.gpsFix = false;
	}

	// while(_SerialGPS.available()) {
	// 	Serial.println(_SerialGPS.readStringUntil('\n'));
	// }

	while(_TheNeoGps.available(_SerialGPS)) {
		_TheFix = _TheNeoGps.read();
		if(_TheFix.valid.location) {
			_LastFixed = now;
			_TheScreenInfo.gpsFix = true;
			_TheScreenInfo.lat_deg = _TheFix.latitude();
			_TheScreenInfo.lon_deg = _TheFix.longitude();
			DrawScreen();
			_FixedLoops++;
		}
		if(_TheFix.valid.speed) {
			_TheScreenInfo.speed_kph = _TheFix.speed_kph();
		}
		if(_TheFix.valid.altitude) {
			_TheScreenInfo.alt_m = _TheFix.altitude();
		}
		if(_TheFix.valid.date && _TheFix.valid.time) {
			_TheScreenInfo.gps_time = (NeoGPS::clock_t)_TheFix.dateTime;
		}
	}
	if(!_TheFix.valid.location) {
		_TheScreenInfo.gpsFix = false;
	}

	LedControl();

	if(UpdateNeeded(now, dist) && _FixedLoops >= FIXED_LOOPS_B4_PUBLISH) {
		LogMsg(Utils::string_format("Position Update Needed!"));
		AwakeModem();

		_lastProcessMillis = now;
		if(dist>MAX_DIST_OK) {
			LogMsg(Utils::string_format("Last Point is farther than max distance allowed!! Ignoring it"));
			_LastPublishedPos.lat_deg = _TheFix.latitude();
			_LastPublishedPos.lon_deg = _TheFix.longitude();
			_LastPublishTime = now;
		}
		else if(verifyGPRSConnection() && verifyMqttConnection()) {
			float volts = (_LastVoltageRead * MAX_VOLTAGE) / (float)MAX_VOLTAGE_READ;
			std::string msg = Utils::string_format("%d,%d,%f,%f,%d,%3.1f,%1.3f,%3.1f", TRACKER_ID, (NeoGPS::clock_t)_TheFix.dateTime,
				_TheFix.latitude(), _TheFix.longitude(), _TheFix.altitude_cm()/100, _TheFix.speed_kph(), volts, dist);
			LogMsg(Utils::string_format("Publishing [%s]", msg.c_str()));
			// if(CONTROL_VOLTAGE) {
			// 	float volts = (_LastVoltageRead * MAX_VOLTAGE) / (float)MAX_VOLTAGE_READ;
			// 	_ThePubSub.publish(FEED_BATTERY, Utils::string_format("%2.2f", volts).c_str(), true);
			// }
			if(_ThePubSub.publish(FEED_BABYTRACKER, msg.c_str(), true)) {
				_TheScreenInfo.mqttState = Utils::string_format("(%3.2f,%3.2f)", _TheFix.latitude(), _TheFix.longitude());
				_LastPublishedPos.lat_deg = _TheFix.latitude();
				_LastPublishedPos.lon_deg = _TheFix.longitude();
				_LastPublishTime=now;
				_FixedLoops=0;
			}
			else {
				_TheScreenInfo.mqttState = "Publish Error";
			}
		}
		else {
			LogMsg(Utils::string_format("GPRS Not Ready :("));
			_GprsErrorCount++;
			_modemGSM.poweroff();
			_ModemInitialized=false;
			if(_GprsErrorCount >= MAX_GPRS_ERRORS) {
				_GprsErrorCount = 0;
				RestartNOW();
			}
		}
	}
	else if(!_TheFix.valid.location && (now - _LastPublishTime) > FORCE_RESET_TIME) {
		LogMsg(Utils::string_format("%dms without GPS!! RESETING"));
		RestartNOW();
	}
	if(_ModemInitialized && _ThePubSub.connected()) {
		_ThePubSub.loop(); //allow the pubsubclient to process incoming messages
	}
	DrawScreen();

	if((now - _LastPublishTime) > SLEEPMODEM_NOUPT && _ModemInitialized && (now - _lastModemON) > UPDATE_TIME*2) {
		LogMsg(Utils::string_format("%dms without a GPS update! Powering off the modem...", (int)(now - _LastPublishTime)));
		SleepModem();
	}

	// if(_TheFix.valid.location && _LastPublishTime > 0 && (now - _lastLightSleep) > UPDATE_TIME && (now - _LastPublishTime) > UPDATE_TIME && _FixedLoops > FIXED_LOOPS_B4_PUBLISH * 2) {
	// 	LogMsg(Utils::string_format("Entering Light Sleep"));
	// 	delay(4000);
	// 	// gpio_hold_en(GPIO_NUM_12);
	// 	// gpio_deep_sleep_hold_en();
	// 		//Turn off everything...
	// 	digitalWrite(PIN_ENABLE, LOW);
	// 	esp_sleep_enable_timer_wakeup(LSLEEP_SECS * uS_TO_S_FACTOR);
	// 	esp_light_sleep_start();
	// 	_lastLightSleep = millis();
	// 	_FixedLoops=0;
	// 		//Turn off everything...
	// 	digitalWrite(PIN_ENABLE, HIGH);
	// }

	delay(50); //we dont want to run all the time? or we do?
}

void DrawScreen()
{
	if(!_ScreenActive || !_ScreenInitialized) {
		return;
	}

	uint16_t charH = _u8g2.getMaxCharHeight() == 0 ? 10 : _u8g2.getMaxCharHeight();
	//uint16_t maxHeight = _u8g2.getHeight();
	uint16_t j = charH;

	std::tm* ptm = std::localtime(&_TheScreenInfo.gps_time);
	char tbuffer[32];
	std::strftime(tbuffer, sizeof(tbuffer), "%d/%m/%Y %H:%M:%S", ptm);

	float volts = (_LastVoltageRead * MAX_VOLTAGE) / (float)MAX_VOLTAGE_READ;
	_u8g2.firstPage();
	do {
		j = charH;
		_u8g2.setCursor(0, j); j += charH * 2;
		if(_TheScreenInfo.gpsFix) {
			_u8g2.print("GPS: F I X E D");
		}
		else {
			_u8g2.print("GPS: Locating...");
		}
		_u8g2.setCursor(0, j); j += charH;
		_u8g2.print(Utils::string_format("Lat/Lon: (%3.3f, %3.3f)", _TheScreenInfo.lat_deg, _TheScreenInfo.lon_deg).c_str());
		_u8g2.setCursor(0, j); j += charH;
		_u8g2.print(Utils::string_format("Alt: %3.1fm Spd: %3.1fkph", _TheScreenInfo.alt_m, _TheScreenInfo.speed_kph).c_str());
		_u8g2.setCursor(0, j); j += charH;
		_u8g2.print(Utils::string_format("GSM: %s", _TheScreenInfo.wifiState.c_str()).c_str());
		_u8g2.setCursor(0, j); j += charH;
		_u8g2.print(Utils::string_format("Mqtt: %s", _TheScreenInfo.mqttState.c_str()).c_str());
		if(CONTROL_VOLTAGE) {
			_u8g2.setCursor(0, j);
			if(_LastVoltageRead>MAX_VOLTAGE_READ-50) {
				_u8g2.print("Battery: Charging/Full");
			}
			else {
				_u8g2.print(Utils::string_format("Battery: %d - %2.2fv", _LastVoltageRead, volts).c_str());
			}
		}
		j += charH;
		_u8g2.setCursor(0, j);
		_u8g2.print(Utils::string_format("Time: %s", tbuffer).c_str());

	} while(_u8g2.nextPage());
}

void RestartNOW()
{
	_TheScreenInfo.wifiState = "RESET";
	DrawScreen();
	delay(3000);
	digitalWrite(PIN_ENABLE, LOW);
	_modemGSM.poweroff();
	ESP.restart();
}

void GoDeepSleep()
{
	LogMsg(Utils::string_format("Going to Sleep!!"));

	_modemGSM.poweroff();
	if(_ScreenActive && _ScreenInitialized) {
		_u8g2.setPowerSave(1);
	}

	esp_sleep_enable_timer_wakeup(SLEEP_TIME_SECS * uS_TO_S_FACTOR);
	esp_deep_sleep_start();
}

void LedControl()
{
	auto now=millis();
	auto sinceChange = (now - _lastLedChange);

	if(_LedON && sinceChange > LED_BLINK_TIME) {
		digitalWrite(PIN_LED, LOW);
		_LedON = false;
		_lastLedChange=now;
		_LedBlinks++;
	}
	else if(!_LedON) {
		bool turnOn = false;
		if(_GprsErrorCount > 0 && _LedBlinks<BLINKS_GSMERROR && sinceChange > LED_BLINK_TIME*2) {
			turnOn = true;
		}
		else if(!_TheFix.valid.location && _LedBlinks<BLINKS_LOCATING && sinceChange > LED_BLINK_TIME*2) {
			turnOn = true;
		}
		else if(_TheFix.valid.location && _LedBlinks<BLINKS_FIXED && sinceChange > LED_BLINK_TIME*2) {
			turnOn = true;
		}
		if(!turnOn && sinceChange > LED_BLINK_OFF) {
			_LedBlinks=0;
			turnOn=true;
		}
		if(turnOn) {
			_LedON = true;
			digitalWrite(PIN_LED, HIGH);
			_lastLedChange=now;
		}
	}
}

bool EnableModem()
{
		// Some start operations
	setupModem();

	//Turn ON Status LED. It will be automatically turned off at some point later
	digitalWrite(PIN_LED, HIGH);

	// Set GSM module baud rate and UART pins
	Serial1.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);

	// Restart takes quite some time
	// To skip it, call init() instead of restart()
	LogMsg(Utils::string_format("Initializing modem..."));
	_modemGSM.restart();

	String modemInfo = _modemGSM.getModemInfo();
	LogMsg(Utils::string_format("Modem: %s", modemInfo.c_str()));

	//Initialize GSM
	LogMsg(Utils::string_format("Initializing GSM/WiFi..."));
	return setupGSM();
}

void TurnModemNetlight(bool on)
{
	if(on) {
		_modemGSM.sendAT("+CNETLIGHT=1");
	}
	else {
		_modemGSM.sendAT("+CNETLIGHT=0");
	}
}

void setupModem()
{
#ifdef MODEM_RST
		// Keep reset high
	pinMode(MODEM_RST, OUTPUT);
	digitalWrite(MODEM_RST, HIGH);
#endif

	pinMode(MODEM_PWRKEY, OUTPUT);
	pinMode(MODEM_POWER_ON, OUTPUT);

	// Turn on the Modem power first
	digitalWrite(MODEM_POWER_ON, HIGH);

	// Pull down PWRKEY for more than 1 second according to manual requirements
	digitalWrite(MODEM_PWRKEY, HIGH);
	delay(100);
	digitalWrite(MODEM_PWRKEY, LOW);
	delay(1000);
	digitalWrite(MODEM_PWRKEY, HIGH);

	// Initialize the indicator as an output
	pinMode(LED_GPIO, OUTPUT);
	digitalWrite(LED_GPIO, LED_OFF);
}

bool setupGSM()
{
	LogMsg(Utils::string_format("Setup GSM..."));

	//aguarda network
	_TheScreenInfo.wifiState = "Connecting...";
	DrawScreen();

	if(!_modemGSM.waitForNetwork(20000)) {
		LogMsg(Utils::string_format("Failed to connect to network"));
		if(!_modemGSM.restart()) {
			LogMsg(Utils::string_format("Restarting GSM - Modem failed"));
		}
		else {
			LogMsg(Utils::string_format("GSM Modem restarted"));
		}

		return false;
	}
	LogMsg(Utils::string_format("Modem registered to network OK!!"));
	_TheScreenInfo.wifiState = "Network OK";
	DrawScreen();

	//connects with GPRS
	if(!_modemGSM.gprsConnect(APN)) {
		LogMsg(Utils::string_format("GPRS Connection Failed :("));
		_TheScreenInfo.wifiState = "GPRS Error";
		DrawScreen();

		return false;
	}
	_TheScreenInfo.wifiState = "GPRS OK";
	LogMsg(Utils::string_format("GPRS Connected OK!!"));
	DrawScreen();

	return true;
}

// verifica se o SIM800L se desconectou, se sim tenta reconectar
bool verifyGPRSConnection()
{
	bool result = false;

	if(_modemGSM.isGprsConnected()) {
		result = true;
		LogMsg(Utils::string_format("GPRS: Connected!"));
		_TheScreenInfo.wifiConnected = true;
		_TheScreenInfo.wifiState = "GPRS OK";
	}
	else {
		LogMsg(Utils::string_format("GPRS: Disconnected. Reconnecting..."));
		_TheScreenInfo.wifiConnected = false;

		if(!_modemGSM.waitForNetwork()) {
			LogMsg(Utils::string_format("Network Failed"));
			_modemGSM.restart();
			if(!_modemGSM.waitForNetwork()) {
				_TheScreenInfo.wifiState = "Net Error";
				return false;
			}
			else {
				LogMsg(Utils::string_format("Modem registered to network OK"));
				_TheScreenInfo.wifiState = "Net OK";
			}
		}
		if(!_modemGSM.gprsConnect(APN)) {
			LogMsg(Utils::string_format("GPRS Failed"));
			_TheScreenInfo.wifiState = "GPRS Error";
		}
		else {
			result = true;
			LogMsg(Utils::string_format("GPRS Connection OK"));
			_TheScreenInfo.wifiConnected = true;
			_TheScreenInfo.wifiState = "GPRS OK";
		}
	}
	if(result) {
		TurnModemNetlight(false);
	}
	return result;
}

bool verifyMqttConnection()
{
	bool ret = true;

	if(!_ThePubSub.connected()) {
		_TheScreenInfo.mqttConnected = false;
		_TheScreenInfo.mqttState = "Connecting...";
		DrawScreen();
		LogMsg(Utils::string_format("PubSubClient Not Connected :("));

		_ThePubSub.setClient(_clientGSM);//_TheWifi);
		_ThePubSub.setServer(MQTT_BROKER, MQTT_PORT); //ADAFRUIT_SERVER, ADAFRUIT_PORT
		_ThePubSub.setKeepAlive(60);
		//_ThePubSub.setCallback(PubSubCallback);
		if(!_ThePubSub.connect("PChanMQTT")) { //, ADAIO_USER, ADAIO_KEY)) {
			LogMsg(Utils::string_format("ERROR!! PubSubClient was not able to connect to broker!!")); //AdafruitIO
			//_TheDebug.NewLine("MQTT error :(");
			_TheScreenInfo.mqttState = Utils::string_format("Error. State=%d", _ThePubSub.state());
			ret = false;
		}
		else {
			_TheScreenInfo.mqttConnected = true;
			_TheScreenInfo.mqttState = "Connected";
			LogMsg(Utils::string_format("PubSubClient connected to broker!!"));
			//_TheDebug.NewLine("MQTT OK!!");
		}
	}
	else {
		_TheScreenInfo.mqttConnected = true;
		_TheScreenInfo.mqttState = "Connected";
	}

	return ret;
}

uint16_t ReadAvgValue(uint8_t pin)
{
	if(CONTROL_VOLTAGE && (millis() - _lastVoltageUpd) > 10000) {
		uint8_t numreads = 2, count = 0;
		uint32_t value = 0;

		_lastVoltageUpd = millis();
		do {
			value += analogRead(pin);
			delay(20);
			count++;
		} while(count < numreads);
		_AccumVoltage += (value / numreads);
		_NumVoltageReadings++;

		_LastVoltageRead = _AccumVoltage / _NumVoltageReadings;
		if(_NumVoltageReadings > 10) {
			_NumVoltageReadings = 1;
			_AccumVoltage = _LastVoltageRead;
		}
		LogMsg(Utils::string_format("Bat=%d Location=%s", _LastVoltageRead, _TheScreenInfo.gpsFix?"Fixed":"Fixing..."), false);
	}

	return _LastVoltageRead;
}

bool UpdateNeeded(unsigned long now, float &dist)
{
	if((now - _LastUpdCheck) < CHECK_EVERY) {
		return false;
	}

	_LastUpdCheck = now;

	if(!_TheFix.valid.location) {
		return false;
	}

	if(_LastPublishTime == 0 || _lastProcessMillis==0) {
		dist=-1.0;
		return true;
	}

	bool result = false;
	unsigned long ellapsed = now - _lastProcessMillis;

	dist=Utils::DistanceBetween2Points(_TheFix.latitude(), _TheFix.longitude(), _LastPublishedPos.lat_deg, _LastPublishedPos.lon_deg);

	if(dist>DIST_UPDATE_3 && ellapsed > UPDATE_TIME_3) {
		LogMsg(Utils::string_format("GPS UPDATE 3. Dist=%f Ellapsed=%d", dist, ellapsed));
		result = true;
	}
	else if(dist > DIST_UPDATE_2 && ellapsed > UPDATE_TIME_2) {
		LogMsg(Utils::string_format("GPS UPDATE 2. Dist=%f Ellapsed=%d", dist, ellapsed));
		result = true;
	}
	else if(dist > DIST_UPDATE_1 && ellapsed > UPDATE_TIME_1) {
		LogMsg(Utils::string_format("GPS UPDATE 1. Dist=%f Ellapsed=%d", dist, ellapsed));
		result = true;
	}
	else if(dist > MIN_DIST_UPDATE && ellapsed > UPDATE_TIME) {
		LogMsg(Utils::string_format("GPS UPDATE NORMAL. Dist=%f Ellapsed=%d", dist, ellapsed));
		result = true;
	}

	if(result && dist<0.5) {
		_CountSameLoc++;
		LogMsg(Utils::string_format("Same position!!! Dist=%f", dist));
		if(_CountSameLoc > MAX_SAME_LOCS) { //this is very strange!!!
			LogMsg(Utils::string_format("Max number of same gps locs. Are we bugged? Reset!"));
			delay(2000);
			ESP.restart();
		}
	}
	else if(result) {
		_CountSameLoc=0;
	}
	if(!result && ellapsed>UPDATE_AT_LEAST) {
		result=true;
	}

	LogMsg(Utils::string_format("Distance from last published point=%3.2f. FixesLoops=%d. CPU=%dMHz/%dMHz", dist, _FixedLoops, ESP.getCpuFreqMHz(), getCpuFrequencyMhz()), false);

	return result;
}

void AwakeModem()
{
	if(!_ModemInitialized) {
			//Turn off everything...
		digitalWrite(PIN_ENABLE, LOW);
		if(EnableModem()) {
			_ModemInitialized=true;
			_GprsErrorCount=0;
			_lastModemON=millis();
			TurnModemNetlight(false);
		}
		else {
			_GprsErrorCount++;
			if(_GprsErrorCount >= MAX_GPRS_ERRORS) {
				_TheScreenInfo.wifiState="GSM ERROR. RESET!!";
				DrawScreen();
				SleepModem();
				delay(1000);
				ESP.restart();
			}
		}
			//Turn on everything...
		digitalWrite(PIN_ENABLE, HIGH);
	}
}

void SleepModem()
{
//	bool res;

	_ModemInitialized=false;
	_clientGSM.stop();
	_modemGSM.gprsDisconnect();
	//_modemGSM.sleepEnable();
	_modemGSM.poweroff();

	// delay(100);

	// // test modem response , res == 0 , modem is sleep
	// res = _modemGSM.testAT();
	// if(res==0) {
	// 	LogMsg(Utils::string_format("MODEM IS NOW OFF"));
	// }
	// else {
	// 	LogMsg(Utils::string_format("MODEM IS STILL AWAKE"));
	// }

//	Serial.println("Use DTR Pin Wakeup");
//	pinMode(MODEM_DTR, OUTPUT);
	//Set DTR Pin low , wakeup modem .
//	digitalWrite(MODEM_DTR, LOW);
}

void ReadFromSerial()
{
	if((Serial.available() > 0)) {
		auto msg = Serial.readStringUntil('\n');

		LogMsg(Utils::string_format("Read from Serial: [%s]", msg.c_str()), false);

		msg.trim();
		msg.toUpperCase();
		if(strcmp("VIEWLOG", msg.c_str())==0) {
			LogMsg("SHOWING THE LOG!!", false);
			_TheLog.seek(0);
			while(_TheLog && _TheLog.available()) {
				Serial.write(_TheLog.readString().c_str());
			}
		}
		else if(strcmp("RESET", msg.c_str()) == 0) {
			LogMsg("Reseting!!", false);
			delay(2000);
			ESP.restart();
		}
		else if(strcmp("MODEMON", msg.c_str()) == 0) {
			if(_ModemInitialized) {
				LogMsg("The modem is already ON!!", false);
			}
			LogMsg("Turning the modem ON!!", false);
			if(EnableModem()) {
				LogMsg("Modem initialized OK!!", false);
				_ModemInitialized = true;
				_lastModemON=millis();
				_GprsErrorCount = 0;
			}
			else {
				LogMsg("Modem FAILED to initialize!!", false);
				_ModemInitialized = false;
				_GprsErrorCount++;
			}
		}
		else if(strcmp("MODEMOFF", msg.c_str()) == 0) {
			LogMsg("Putting the modem to sleep...", false);
			SleepModem();
		}
		else if(strcmp("PUBLISH", msg.c_str()) == 0) {
			float volts = (_LastVoltageRead * MAX_VOLTAGE) / (float)MAX_VOLTAGE_READ;
			std::string msg = Utils::string_format("%d,%d,%f,%f,%d,%3.1f,%1.3f,%3.1f", TRACKER_ID, (NeoGPS::clock_t)_TheFix.dateTime,
				_TheFix.latitude(), _TheFix.longitude(), _TheFix.altitude_cm() / 100, _TheFix.speed_kph(), volts, -2);
			LogMsg(Utils::string_format("Publishing [%s]...", msg.c_str()), false);
			if(_ThePubSub.publish(FEED_BABYTRACKER, msg.c_str(), true)) {
				LogMsg("Publish OK", false);
			}
			else {
				LogMsg("Publish FAILED", false);
			}
		}
		else if(strcmp("SLEEP", msg.c_str()) == 0) {
			LogMsg("Going LightSleep for 60s", false);
			delay(1000);
			esp_sleep_enable_timer_wakeup(60 * uS_TO_S_FACTOR);
			esp_light_sleep_start();
			LogMsg("Awoken!!", false);
		}
		else if(strcmp("DEEPSLEEP", msg.c_str()) == 0) {
			LogMsg("Going DeepSleep for 60s", false);
			esp_sleep_enable_timer_wakeup(60 * uS_TO_S_FACTOR);
			esp_deep_sleep_start(); //this call does not return
		}
		else if(strcmp("GPRSON", msg.c_str()) == 0) {
			LogMsg("Verifying GPRS/MQTT connection", false);
			if(verifyGPRSConnection() && verifyMqttConnection()) {
				LogMsg("GPRS && MQTT OK", false);
			}
			else {
				LogMsg("Conexion FAILED :(", false);
			}
		}
		else {
			LogMsg("UNKNOWN COMMAND", false);
		}
	}
}

void LogMsg(std::string msg, bool add2log)
{
	log_d("%s", msg.c_str());

	if(add2log && _LogAvailable && _TheLog) {
		_TheLog.seek(0, fs::SeekEnd);
		_TheLog.println(msg.c_str());
	}
}
