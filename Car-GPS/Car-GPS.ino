#include <SIM808.h>
#include <ArduinoLog.h>
#include <SoftwareSerial.h>



#define SIM_RX		2	///< SIM808 RXD
#define SIM_TX		3	///< SIM808 TXD
#define SIM_RST		4	///< SIM808 RESET
#define SIM_PWR		5	///< SIM808 PWRKEY
#define SIM_STATUS	6 // SIM808_UNAVAILABLE_PIN ///< SIM808 STATUS

#define SIM808_BAUDRATE 9600    ///< Control the baudrate use to communicate with the SIM808 module
#define SERIAL_BAUDRATE 9600    ///< Controls the serial baudrate between the arduino and the computer
#define NO_FIX_GPS_DELAY 3000   ///< Delay between each GPS read when no fix is acquired
#define FIX_GPS_DELAY  10000    ///< Delay between each GPS read when a fix is acquired
#define NETWORK_DELAY  60        ///< Delay between each GPS read in minutes when car is moving
#define BUFFER_SIZE 512         ///< Size of the buffer on the main program side
#define NL "\n"
#define POSITION_SIZE   128 
#define MAX_MOVEMENT    0.000001 //0.0001

#define GPRS_APN    "internet"  ///< Your provider Access Point Name
#define GPRS_USER   NULL        ///< Your provider APN user (usually not needed)
#define GPRS_PASS   NULL        ///< Your provider APN password (usually not needed)

#define NUM_SAMPLES 10
#define LOW_CAR_BATTERY_SLEEP_CYCLES 15
#define BATTERY_CALIBRATE 3.3
#define CAR_BAT_CALIBRATE 3.53
#define BATTERY_CAR_LOW  11.5

SoftwareSerial simSerial = SoftwareSerial(SIM_TX, SIM_RX);
SIM808 sim808 = SIM808(SIM_RST, SIM_PWR, SIM_STATUS);
char buffer[BUFFER_SIZE];
bool done = false;
char position[POSITION_SIZE];
float last_lon = 0.0;
float last_lat = 0.0;
int SleepCycles = LOW_CAR_BATTERY_SLEEP_CYCLES; // Force a run after poweron
int delay_sec;
int time;

void setup() {
	Serial.begin(SERIAL_BAUDRATE);
	Log.begin(LOG_LEVEL_NOTICE, &Serial);

	simSerial.begin(SIM808_BAUDRATE);
	sim808.begin(simSerial);

	Log.notice(F("Powering on SIM808..." NL));
	sim808.powerOnOff(true);
	sim808.init();
}

void loop()
{
	Log.notice(".");
	uint16_t numsat;
	float lat, lon, alt;
	char* date;
	__FlashStringHelper* state;
	bool s;
	unsigned int sum_B = 0;                    // sum of samples taken
	unsigned int sum_C = 0;                    // sum of samples taken
	unsigned char sample_count = 0; // current sample number

	while (sample_count < NUM_SAMPLES) {
		sum_B += analogRead(A0);
		sum_C += analogRead(A1);
		sample_count++;
		delay(10);
	}

	float voltage_Bat = BATTERY_CALIBRATE * ((float)sum_B / (float)NUM_SAMPLES * 5.015) / 1024.0;
	float voltage_Car = CAR_BAT_CALIBRATE * ((float)sum_C / (float)NUM_SAMPLES * 5.015) / 1024.0;

	Log.notice(F("Voltage Bat, Car : %D, %D" NL), voltage_Bat, voltage_Car);

	if (voltage_Car < BATTERY_CAR_LOW && SleepCycles < LOW_CAR_BATTERY_SLEEP_CYCLES)
	{
		Log.notice(F("Car is low on battery..(dryrun %i/%i)"  NL), SleepCycles+1, LOW_CAR_BATTERY_SLEEP_CYCLES);
		delay_sec = 60; // Default sleeptime
		SleepCycles++;
		sim808.powerOnOff(false);  // Powerdown GPS and Radio
		digitalWrite(SIM_PWR, LOW);
		sim808.reset();

	}
	else
	{
		SleepCycles = 0;
		Log.notice(F("Powering on GPS..." NL));

		sim808.powerOnOff(true);
		sim808.powerOnOffGps(true);

		delay(1000);

		SIM808_GPS_STATUS gps_status = sim808.getGpsStatus(position, POSITION_SIZE, 4);

		while (gps_status < SIM808_GPS_STATUS::ACCURATE_FIX)
		{
			sim808.powerOnOffGps(true);
			Log.notice(F("No fix yet..." NL));
			delay(NO_FIX_GPS_DELAY);
			gps_status = sim808.getGpsStatus(position, POSITION_SIZE, 4);
		}

		sim808.enableGprs(GPRS_APN, GPRS_USER, GPRS_PASS);

		sim808.getGprsPowerState(&s);
		if (s)
			Log.notice(F("GPRS already powered on..." NL));
		else
		{
			Log.notice(F("Powering on GPRS..." NL));
			sim808.enableGprs(GPRS_APN, GPRS_USER, GPRS_PASS);
		}

		bool isAvailable = true;

		do {
			SIM808_NETWORK_REGISTRATION_STATE status = sim808.getNetworkRegistrationStatus();
			isAvailable = static_cast<int8_t>(status) &
				(static_cast<int8_t>(SIM808_NETWORK_REGISTRATION_STATE::REGISTERED) | static_cast<int8_t>(SIM808_NETWORK_REGISTRATION_STATE::ROAMING))
				!= 0;

			Log.notice(F("isAvailable %i..." NL), isAvailable);

			if (!isAvailable) {
				Log.notice(F("No network yet..." NL));
				delay(2000);  // wait 2 sec for radio to start
			}
		} while (!isAvailable);


		sim808.getGpsField(position, SIM808_GPS_FIELD::GNSS_USED, &numsat);
		sim808.getGpsField(position, SIM808_GPS_FIELD::LATITUDE, &lat);
		sim808.getGpsField(position, SIM808_GPS_FIELD::LONGITUDE, &lon);
		sim808.getGpsField(position, SIM808_GPS_FIELD::ALTITUDE, &alt);
		sim808.getGpsField(position, SIM808_GPS_FIELD::UTC, &date);

		float movement = sqrt(pow(lat - last_lat, 2) + pow(lon - last_lon, 2));
		last_lat = lat; last_lon = lon;
		Log.notice(F("Movement = %D" NL), movement * 1000);

		Log.notice(F("Sending HTTP request..." NL));
		char b[20];

		time = atoi(&date[12]);

		strcpy(buffer, "time="); strncat(buffer, date, 14);
		strcat(buffer, "&numsat="); itoa(numsat, b, 10); strcat(buffer, b);
		strcat(buffer, "&lat="); dtostrf(lat, 3, 6, b); strcat(buffer, b);
		strcat(buffer, "&lon="); dtostrf(lon, 3, 6, b); strcat(buffer, b);
		strcat(buffer, "&alt="); dtostrf(alt, 3, 6, b); strcat(buffer, b);
		strcat(buffer, "&bat="); dtostrf(voltage_Bat, 3, 2, b); strcat(buffer, b);
		strcat(buffer, "&car="); dtostrf(voltage_Car, 3, 2, b); strcat(buffer, b);

		Log.notice(F("Buffer1 : %s" NL), buffer);

		//notice that we're using the same buffer for both body and response
		uint16_t responseCode = sim808.httpPost("http://62.242.176.149/GPS/gps.php", F("application/x-www-form-urlencoded"), buffer, buffer, BUFFER_SIZE);

		Log.notice(F("Buffer2 : %s" NL), buffer);

		Log.notice(F("Server responsed : %d" NL), responseCode);
		if (responseCode == 601)
		{
			sim808.powerOnOff(false);
			sim808.reset();
			delay_sec = 10;
			SleepCycles = LOW_CAR_BATTERY_SLEEP_CYCLES; // Force a new run
		}
		else
			delay_sec = (NETWORK_DELAY - time);
	}

	Log.notice(F("sleeping %i sec" NL), delay_sec);
	delay(delay_sec * 1000l);

}

