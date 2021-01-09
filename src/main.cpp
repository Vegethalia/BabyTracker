
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
//#include "SharedUtils/ScreenDebugger.h" //to use the screen as a debugger

#define ALWAYS_ON        true
#define CONTROL_VOLTAGE  true //if true, PIN_BAT will be used to read the voltage of the battery

#define MAX_VOLTAGE      4.32f //when charging
#define MIN_VOLTAGE      3.40f
#define MAX_VOLTAGE_READ 2225  //when charging

#define LED_BLINK_TIME   150
#define LED_BLINK_OFF    (LED_BLINK_TIME*10)
#define BLINKS_LOCATING  1
#define BLINKS_FIXED     2
#define BLINKS_GSMERROR  3

#define SIM800L_IP5306_VERSION_20200811
#include "SharedUtils/lilygo_utils.h"

#define PIN_GPS_TX  18
#define PIN_GPS_RX  19
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22
#define PIN_ENABLE  12
#define PIN_BAT     35 //15
#define PIN_LED     2

#define SCREEN_WIDTH   128 // OLED display width, in pixels
#define SCREEN_HEIGHT  64 // OLED display height, in pixels
#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for micro seconds to seconds

// ADAFRUIT Feeds / Related stuff
// #define ADAFRUIT_ADDR      "52.7.124.212" //"io.adafruit.com"
// #define ADAFRUIT_PORT      1883
// #define FEED_LOCATION      "/feeds/location/csv" //sensor_value,latitude,longitude,elevation

#define TRACKER_ID       1 //this is the 1st tracker!
#define MQTT_BROKER      "mitotoro.synology.me"
#define MQTT_PORT        1888
#define FEED_BABYTRACKER "babytracker/loc" //ID,datetime,latitude,longitude,elevation,speed
#define FEED_BATTERY     "babytracker/bat" //ID,datetime,latitude,longitude,elevation,speed

//U8G2_SH1106_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, PIN_I2C_SCL, PIN_I2C_SDA);
U8G2_SSD1306_128X64_NONAME_1_HW_I2C _u8g2(U8G2_R0, PIN_I2C_SCL, PIN_I2C_SDA);

#define MAX_GPRS_ERRORS        5 //after this errors, restart everything
#define FIXED_LOOPS_B4_PUBLISH 3 //once we have a fix, wait this loops before trying to publish the value

#define TINY_GSM_MODEM_SIM800 // Modem difinition... we are using this one (SIM800L)
#define TINY_GSM_DEBUG        Serial
#include <TinyGsmClient.h>


SoftwareSerial  _SerialGPS(PIN_GPS_RX, PIN_GPS_TX); // no more hw serials available. We have to use a software serial.
TinyGsm         _modemGSM(Serial1); //With LILYGO TCALL the modem rx/tx is this one
TinyGsmClient   _clientGSM(_modemGSM);

//TwoWire         _I2Cscreen = TwoWire(0);
//WiFiClient      _TheWifi;
PubSubClient    _ThePubSub;

//ScreenDebugger  _TheDebug(&_u8g2, 5, 1);
ScreenInfo      _TheScreenInfo;
bool            _ScreenActive = false;

NMEAGPS         _TheNeoGps; // This object parses the GPS characters
gps_fix         _TheFix;    // This global structure contains the GPS fix returned by _TheNeoGps

//TIMING VARS
unsigned long _delayTimeUpt = 60000;
unsigned long _lastProcessMillis = 0;
unsigned long _LastFixed = 0;
unsigned long _lastLedChange = 0;
uint8_t       _GprsErrorCount = 0;
int           _timeout = 0;
uint16_t      _sleepForSecs = 60;
uint16_t      _FixedLoops = 0;
bool          _ModemInitialized = false;
bool          _ScreenInitialized = false;
bool          _LedON = false;
uint8_t       _LedBlinks=0; //Counts the number of led blinks since the last _LedBlinks=0
uint16_t      _LastVoltageRead = MAX_VOLTAGE_READ;
unsigned long _lastVoltageUpd = 0;
unsigned long _AccumVoltage=0;
uint16_t      _NumVoltageReadings=0;

//FORWARD DECLARATIONS
void DrawScreen();
void setupModem();
bool setupGSM();
bool verifyGPRSConnection();
bool verifyMqttConnection();
void GoDeepSleep();
bool EnableModem();
void LedControl();
uint16_t ReadAvgValue(uint8_t pin);
//END FF DECLARATIONS

void setup()
{
	Serial.begin(115200);
	// wait for serial monitor to open
	while(!Serial);

	log_d("Setup Serial GPS...");
	_SerialGPS.begin(9600);

	log_d("Enabling transistor...");
	pinMode(PIN_ENABLE, OUTPUT); //We have the gps and the screen as the load of a transistor, so they fully disconnect when esp32 goes deep sleep
	digitalWrite(PIN_ENABLE, HIGH);
	pinMode(PIN_LED, OUTPUT);
	digitalWrite(PIN_LED, LOW);
	// if(CONTROL_VOLTAGE) {
	// 	pinMode(PIN_BAT, INPUT); //not really needed, all pins start as input
	// 	analogReadResolution(10);
	// 	analogSetPinAttenuation(PIN_BAT, ADC_11db); //max input value ~2600mv ????
	// }

	// Start power management
	if(setupPMU() == false) {
		Serial.println("Setting power error");
	}

	//We will setup the modem only when we have a gps fix, so we can safe battery

	log_d("Begin Display...");
	// if(!_I2Cscreen.begin(PIN_I2C_SDA, PIN_I2C_SCL, 100000)) { //0=default 100000 100khz
	// 	log_d("I2C bus init error :(");
	// }
//	_u8g2.setBusClock(100000);
	if(_ScreenActive && _u8g2.begin()) {
		_ScreenInitialized=true;
		_u8g2.setFont(u8g2_font_5x8_mf);
		//_TheDebug.SetFont(ScreenDebugger::SIZE1);
		_TheScreenInfo.wifiState = "Sleeping...";
		_TheScreenInfo.mqttState = "Waiting GSM...";
		DrawScreen();
	}
	else {
		log_d("Screen NOT available");
	}

//	_TheDebug.NewLine("Setup Complete!");
	log_d("Setup Complete!");
}

void loop()
{
	if(CONTROL_VOLTAGE && (millis() - _lastVoltageUpd) > 10000) {
		_lastVoltageUpd=millis();
		_AccumVoltage += ReadAvgValue(PIN_BAT);
		_NumVoltageReadings++;
		_LastVoltageRead = _AccumVoltage / _NumVoltageReadings;
		if(_NumVoltageReadings>10) {
			_NumVoltageReadings = 1;
			_AccumVoltage = _LastVoltageRead;
		}
		log_d("Bat=%d", _LastVoltageRead);
	}

	//log_d("Voltage value=%d", (int)(voltageRead));

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
			if(_FixedLoops == 0 && !_ModemInitialized) {
				if(EnableModem()) {
					_ModemInitialized=true;
					_GprsErrorCount=0;
				}
				else {
					_GprsErrorCount++;
					if(_GprsErrorCount >= MAX_GPRS_ERRORS) {
						_TheScreenInfo.wifiState="GSM ERROR. RESET!!";
						DrawScreen();
						delay(3000);
						ESP.restart();
					}
				}
			}
			_FixedLoops++;
//			log_d("Fixed! loop=%d", _FixedLoops);
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

	if(_TheFix.valid.location && (now - _lastProcessMillis) >= _delayTimeUpt && _FixedLoops >= FIXED_LOOPS_B4_PUBLISH) {
		_lastProcessMillis = now;
		log_d("Time 2 update!!");

		if(verifyGPRSConnection() && verifyMqttConnection()) {
			std::string msg = Utils::string_format("%d, %d, %f, %f, %d, %f", TRACKER_ID, (NeoGPS::clock_t)_TheFix.dateTime,
				_TheFix.latitude(), _TheFix.longitude(), _TheFix.altitude_cm()/100, _TheFix.speed_kph());
			log_d("Publishing [%s]", msg.c_str());
			if(CONTROL_VOLTAGE) {
				float volts = (_LastVoltageRead * MAX_VOLTAGE) / (float)MAX_VOLTAGE_READ;
				_ThePubSub.publish(FEED_BATTERY, Utils::string_format("%2.2f", volts).c_str(), true);
			}
			if(_ThePubSub.publish(FEED_BABYTRACKER, msg.c_str(), true)) {
				_TheScreenInfo.mqttState = Utils::string_format("(%3.2f,%3.2f)", _TheFix.latitude(), _TheFix.longitude());
				if(!ALWAYS_ON) {
					_TheScreenInfo.wifiState = "DeepSleep...";
					DrawScreen();
					GoDeepSleep();
				}
				else {
					_FixedLoops=0;
				}
			}
			else {
				_TheScreenInfo.mqttState = "Publish Error";
			}
		}
		else {
			log_d("GPRS Not Ready :(");
			_GprsErrorCount++;
			if(_GprsErrorCount >= MAX_GPRS_ERRORS) {
				_GprsErrorCount = 0;
				_TheScreenInfo.wifiState = "RESET";
				DrawScreen();
				delay(3000);
				ESP.restart();
			}
		}
	}
	if(_ThePubSub.connected()) {
		_ThePubSub.loop(); //allow the pubsubclient to process incoming messages
	}
	DrawScreen();
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

void GoDeepSleep()
{
	log_d("Going to Sleep!!");

	_modemGSM.poweroff();
	_u8g2.setPowerSave(1);

	esp_sleep_enable_timer_wakeup(_sleepForSecs * uS_TO_S_FACTOR);
	esp_deep_sleep_start();
}

void LedControl()
{
	auto now=millis();
	auto sinceChange = (now - _lastLedChange);

	if(_LedON && sinceChange > LED_BLINK_TIME) {
//		log_d("OFF");
		digitalWrite(PIN_LED, LOW);
		_LedON = false;
		_lastLedChange=now;
		_LedBlinks++;
	}
	else if(!_LedON) {
		bool turnOn = false;
		if(_GprsErrorCount > 0 && _LedBlinks<BLINKS_GSMERROR && sinceChange > LED_BLINK_TIME) {
//			log_d("ON ERROR");
			turnOn = true;
		}
		else if(!_TheFix.valid.location && _LedBlinks<BLINKS_LOCATING && sinceChange > LED_BLINK_TIME) {
//			log_d("ON LOCATING");
			turnOn = true;
		}
		else if(_TheFix.valid.location && _LedBlinks<BLINKS_FIXED && sinceChange > LED_BLINK_TIME) {
//			log_d("ON FIXED");
			turnOn = true;
		}
		if(!turnOn && sinceChange > LED_BLINK_OFF) {
//			log_d("ON AFTER OFF");
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
	log_d("Initializing modem...");
	_modemGSM.restart();

	String modemInfo = _modemGSM.getModemInfo();
	log_d("Modem: %d", modemInfo.c_str());

		//Initialize Wifi
	//_TheDebug.NewLine("Initializing WiFi...");
	log_d("Initializing GSM/WiFi...");
	return setupGSM();
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
	log_d("Setup GSM...");

	//aguarda network
	_TheScreenInfo.wifiState = "Connecting...";
	DrawScreen();

	if(!_modemGSM.waitForNetwork(20000)) {
		log_d("Failed to connect to network");
		if(!_modemGSM.restart()) {
			log_d("Restarting GSM\nModem failed");
		}
		else {
			log_d("GSM Modem restarted");
		}

		return false;
	}
	log_d("Modem registered to network OK!!");
	_TheScreenInfo.wifiState = "Network OK";
	DrawScreen();

	//connects with GPRS
	if(!_modemGSM.gprsConnect(APN)) {
		log_d("GPRS Connection Failed :(");
		_TheScreenInfo.wifiState = "GPRS Error";
		DrawScreen();

		return false;
	}
	_TheScreenInfo.wifiState = "GPRS OK";
	log_d("GPRS Connected OK!!");
	DrawScreen();

	return true;
}

// verifica se o SIM800L se desconectou, se sim tenta reconectar
bool verifyGPRSConnection()
{
	bool result = false;

	if(_modemGSM.isGprsConnected()) {
		result = true;
		log_d("GPRS: Connected!");
		_TheScreenInfo.wifiConnected = true;
		_TheScreenInfo.wifiState = "GPRS OK";
	}
	else {
		log_d("GPRS: Disconnected. Reconnecting...");
		_TheScreenInfo.wifiConnected = false;

		if(!_modemGSM.waitForNetwork()) {
			log_d("Network Failed");
			_modemGSM.restart();
			_TheScreenInfo.wifiState = "Net Error";
		}
		else {
			if(!_modemGSM.gprsConnect(APN)) {
				log_d("GPRS Failed");
				_TheScreenInfo.wifiState = "GPRS Error";
			}
			else {
				result = true;
				log_d("GPRS Connection OK");
				_TheScreenInfo.wifiConnected = true;
				_TheScreenInfo.wifiState = "GPRS OK";
			}
		}
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
		log_d("PubSubClient Not Connected :(");

		_ThePubSub.setClient(_clientGSM);//_TheWifi);
		_ThePubSub.setServer(MQTT_BROKER, MQTT_PORT); //ADAFRUIT_SERVER, ADAFRUIT_PORT
		_ThePubSub.setKeepAlive(60);
		//_ThePubSub.setCallback(PubSubCallback);
		if(!_ThePubSub.connect("PChanMQTT")) { //, ADAIO_USER, ADAIO_KEY)) {
			log_d("ERROR!! PubSubClient was not able to connect to broker!!"); //AdafruitIO
			//_TheDebug.NewLine("MQTT error :(");
			_TheScreenInfo.mqttState = Utils::string_format("Error. State=%d", _ThePubSub.state());
			ret = false;
		}
		else {
			_TheScreenInfo.mqttConnected = true;
			_TheScreenInfo.mqttState = "Connected";
			log_d("PubSubClient connected to broker!!");
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
	uint8_t numreads = 2, count = 0;
	uint32_t value = 0;

	do {
		value += analogRead(pin);
		delay(20);
		count++;
	} while(count < numreads);

	return value / numreads;
}
