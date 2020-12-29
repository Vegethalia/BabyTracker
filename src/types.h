#include <string>

struct ScreenInfo {
	float lon_deg;
	float lat_deg;
	float speed_kph;
	float alt_m;
	time_t gps_time;

	bool gpsFix;
	bool wifiConnected;
	bool mqttConnected;
	std::string wifiState;
	std::string mqttState;

	ScreenInfo() { Reset(); }

	void Reset()
	{
		lon_deg = lat_deg = speed_kph = alt_m = 0.0f;
		gpsFix = wifiConnected = mqttConnected = false;
		gps_time=0;
		wifiState.clear();
		mqttState.clear();
	}
};
