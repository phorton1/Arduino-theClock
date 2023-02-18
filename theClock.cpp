
#include "theClock.h"
#include <myIOTLog.h>
#include <Adafruit_NeoPixel.h>

// USES A TASK FOR RUNNING THE CLOCK
//
// there is too much critical timing in the clock run() method to allow it
// allow it to run in loop() on the same core as the webservers, etc.
// When just run from the default core(1), along with the Webserver, it
// regularly misses crossings, etc, esp when the myIOT WebSocket goes on and off.
//
// So I made it a task on the unused core 0. However, tasks are limited to 10ms
// time slices, and so it did not work at first.
//
// I was able to get it working. First I changed the call to vTaskDelay(0)
// as I read that would yield without delay.  No joy.  I set the priority
// of the task down to 1, no joy.  Finally I disabled the Watch Dog Timer,
// (see code below) and that seems to work.


// TUNING STRATEGY
//
// 1. The hall threshold should be high enough so that we don't get false positions.
// 2. With springs all the way out, using STATIC (PID off) mode, tune HIGH and LOW values for reliable motion.
// 3. Adjust the weight to be close, but reliably longer than 1000ms
// 4. Tune springs a bit
// 5. With PID mode, fine adjust the springs and values


//----------------------------
// halls
//----------------------------

#define NUM_HALL_PINS	3
#define NUM_SAMPLES     5

const int hall_pins[NUM_HALL_PINS] =
{
	PIN_HALL1,
	PIN_HALL2,
	PIN_HALL3,
};

static int circ_ptr = 0;
static int circ_buf[NUM_HALL_PINS][NUM_SAMPLES];
static int hall_value[NUM_HALL_PINS];

#if COMPILE_VERSION == 2
	static int hall_zero[NUM_HALL_PINS] = {1795,1876,1836};
#else
	static int hall_zero[NUM_HALL_PINS] = {1734,1876,1867};
#endif


//----------------------------
// motors
//----------------------------

#define PWM_FREQUENCY	5000
#define PWM_RESOLUTION	8


//----------------------------
// pixel(s)
//----------------------------

static Adafruit_NeoPixel pixels(NUM_PIXELS,PIN_LEDS);

static bool show_pixels = false;

void setPixel(int num, uint32_t color)
{
	pixels.setPixelColor(num,color);
	show_pixels = true;
}


//--------------------------------------------
// vars
//--------------------------------------------

// Basics

static bool clock_started = 0;
static int position = 0;
static int max_left = 0;
static int max_right = 0;
static uint32_t cycle_start = 0;
static uint32_t cycle_duration = 0;
static uint32_t last_change = 0;


// PID


static int32_t total_error = 0;
static uint32_t prev_p = 0;
static int pid_power = 0;

// Statistics

static uint32_t num_beats   = 0;
static int num_restarts     = 0;
static int num_stalls_left  = 0;
static int num_stalls_right = 0;
static int32_t low_error    = 0;
static int32_t high_error   = 0;
static int32_t low_dur      = 0;
static int32_t high_dur     = 0;
static int min_power_used 	= 0;
static int max_power_used   = 0;


// Buttons and pixel flashing

#define FUNCTION_NONE  			0
#define FUNCTION_STOPPED		1	// white		state
#define FUNCTION_STOPPING		2	// white		short press
#define FUNCTION_STARTING       3	// green		short press
#define FUNCTION_WIFI_ON        4	// blue			long press (over 2 seconds)
#define FUNCTION_WIFI_OFF       5	// magenta		long press (over 2 seconds)
#define FUNCTION_RESET			6	// red			super long press (over 8 seconds)

static int flash_fxn = FUNCTION_NONE;
static bool flash_on = 0;
static int flash_count = 0;
static uint32_t flash_dur = 0;
static uint32_t flash_time = 0;

static uint32_t button_chk = 0;
static uint32_t button_down = 0;


#if 0
	void one_time_calibrate_hall()
	{
		// assuming pendulum is hanging down
		// set zero for sensors 0 and 1

		Serial.println("calibrating hall pins - pendulum must be stopped, center");
		delay(500);
		for (int i=0; i<NUM_HALL_PINS; i++)
		{
			if (i != 1)
				hall_zero[i] = analogRead(hall_pins[i]);
		}
		delay(500);

		// then set the central sensor

		Serial.println("move pendulum to one side and press any key");
		while (!Serial.available()) { delay(5); }
		int c = Serial.read();
		delay(100);
		hall_zero[2] = analogRead(hall_pins[1]);
		delay(500);
		Serial.print("hall_calibration complete ");
		Serial.print(hall_zero[0]);
		Serial.print(",");
		Serial.print(hall_zero[1]);
		Serial.print(",");
		Serial.println(hall_zero[2]);
			//hall_zero[3],
			//hall_zero[4]);
	}
#endif




// virtual
void theClock::setup()	// override
{
	LOGU("theClock::setup() started");

	pixels.setPixelColor(0,MY_LED_RED);
	pixels.show();
	delay(500);

	pinMode(PIN_BUTTON,INPUT_PULLUP);

	for (int i=0; i<NUM_HALL_PINS; i++)
	{
		pinMode(hall_pins[i],INPUT);
	}

	ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION);
	ledcSetup(1, PWM_FREQUENCY, PWM_RESOLUTION);
	ledcAttachPin(PIN_ENA, 0);
	ledcAttachPin(PIN_ENB, 1);
	ledcWrite(0,0);
	ledcWrite(1,0);
	pinMode(PIN_INA1,OUTPUT);
	pinMode(PIN_INA2,OUTPUT);
	pinMode(PIN_INB1,OUTPUT);
	pinMode(PIN_INB2,OUTPUT);
	digitalWrite(PIN_INA1,0);
	digitalWrite(PIN_INA2,0);
	digitalWrite(PIN_INB1,0);
	digitalWrite(PIN_INB2,0);

	// one_time_calibrate_hall();

	pixels.setPixelColor(0,MY_LED_BLUE);
	pixels.show();
	myIOTDevice::setup();

	pixels.setPixelColor(0,MY_LED_GREEN);
	pixels.show();

	LOGI("starting clockTask");
	xTaskCreatePinnedToCore(clockTask,
		"clockTask",
		8192,           // task stack
		NULL,           // param
		10,  	        // note that the priority is higher than one
		NULL,           // returned task handle
		ESP32_CORE_OTHER);

	if (_clock_running)
		startClock();
	else
		flash_fxn = FUNCTION_STOPPED;

	LOGU("theClock::setup() finished");
	pixels.setPixelColor(0,MY_LED_BLACK);
	pixels.show();

}	// theClock::setup()



void motor(int state, int power)
	// note output to channel B for backward compatibility with V1 circuit board
{
	int use_power = state ? power : 0;
	ledcWrite(0, use_power);
	ledcWrite(1, use_power);
	digitalWrite(PIN_INA1,state == 1  ? 1 : 0);
	digitalWrite(PIN_INA2,state == -1 ? 1 : 0);
	digitalWrite(PIN_INB1,state == 1  ? 1 : 0);
	digitalWrite(PIN_INB2,state == -1 ? 1 : 0);
	// LOGD("motor(%d,%d)",state,power);
}


void init()
{
	clock_started = 0;

	position = 0;
	max_left = 0;
	max_right = 0;
	cycle_start = 0;
	cycle_duration = 0;
	total_error = 0;
	prev_p = 0;

	circ_ptr = 0;
	memset(circ_buf,0,NUM_HALL_PINS*NUM_SAMPLES*sizeof(int));

	num_beats = 0;
	num_restarts = 0;
	num_stalls_left = 0;
	num_stalls_right = 0;
	low_error = 0;
	high_error = 0;
	low_dur = 32767;
	high_dur = 0;
	min_power_used = 255;
	max_power_used = 0;
}


void theClock::clearStats()
{
	LOGU("STATISTICS CLEARED");

	total_error = 0;
	prev_p = 0;

	num_beats = 0;
	num_restarts = 0;
	num_stalls_left = 0;
	num_stalls_right = 0;
	low_error = 0;
	high_error = 0;
	low_dur = 32767;
	high_dur = 0;
	min_power_used = 255;
	max_power_used = 0;

	// resetting the statistics also resets the
	// start time for display.

	if (clock_started)
		the_clock->setTime(ID_TIME_LAST_START,time(NULL));

}



// from chatGPT

#include <WiFi.h>
#include <WiFiUdp.h>

static unsigned int localPort = 2390;      // local port to listen for UDP packets
static const char* ntpServer = "pool.ntp.org";

// modified, theirs was 3600,3600
// so with DST turned on, this gave 7200.
// NOTE that this IS NOT what is displayed in the log time,
// because THAT goes through the time zone settings in myIOTHttp.cpp NTP code ...
// AND I *think* that passing a GMT time to the web browser shows it in the machine's timezone,
// AND NOTE that display of times with the command line is NOT working.


static const long  gmtOffset_sec = 0;		// time(NULL) returns GMT?
static const int   daylightOffset_sec = 0;		// was 3600;

// my additions

static const int NTP_PACKET_SIZE = 48;
static byte packetBuffer[ NTP_PACKET_SIZE];

static WiFiUDP  udp;
IPAddress timeServer;
static bool udp_started = 0;



static void sendNTPpacket(IPAddress &address)
	// send an NTP request to the time server at the given address
{
	// set all bytes in the buffer to 0
	memset(packetBuffer, 0, NTP_PACKET_SIZE);
	// Initialize values needed to form NTP request
	// (see URL above for details on the packets)
	packetBuffer[0] = 0b11100011;   // LI, Version, Mode
	packetBuffer[1] = 0;     // Stratum, or type of clock
	packetBuffer[2] = 6;     // Polling Interval
	packetBuffer[3] = 0xEC;  // Peer Clock Precision
	// 8 bytes of zero for Root Delay & Root Dispersion
	packetBuffer[12]  = 49;
	packetBuffer[13]  = 0x4E;
	packetBuffer[14]  = 49;
	packetBuffer[15]  = 52;

	// all NTP fields have been given values, now
	// you can send a packet requesting a timestamp:

	udp.beginPacket(address, 123); //NTP requests are to port 123
	udp.write(packetBuffer, NTP_PACKET_SIZE);
	udp.endPacket();
}



time_t getNtpTime()
{
	if (WiFi.status() != WL_CONNECTED)
		return 0;
	if (udp_started == -1)
		return 0;

	if (!udp_started)
	{
		udp_started = -1;
		LOGD("starting UDP");
		WiFi.hostByName(ntpServer, timeServer);
		LOGD("%s = %s",ntpServer,timeServer.toString().c_str());
		if (!udp.begin(localPort))
		{
			LOGE("Could not start UDP on localPort(%d)",localPort);
			return 0;
		}
		LOGD("UDP started on localPort(%d)",localPort);
		udp_started = 1;
	}

	while (udp.parsePacket() > 0) ; // discard any previously received packets

	LOGD("Transmit NTP Request");
	sendNTPpacket(timeServer);

	uint32_t beginWait = millis();
	while (millis() - beginWait < 1500)
	{
		int size = udp.parsePacket();
		if (size >= NTP_PACKET_SIZE)
		{
			LOGD("Receive NTP Response");
			udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
			unsigned long secsSince1900;

			// convert four bytes starting at location 40 to a long integer
			secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
			secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
			secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
			secsSince1900 |= (unsigned long)packetBuffer[43];
			secsSince1900 -= 2208988800UL + gmtOffset_sec + daylightOffset_sec;

			LOGD("secsSince1900=%d",secsSince1900);
			return secsSince1900;

		}
	}

	LOGE("No NTP Response :-(");
	return 0; // return 0 if unable to get the time
}




void theClock::startClock()
{
	LOGU("startClock()");

	init();
	clock_started = 1;
	setPixel(PIXEL_MAIN, MY_LED_WHITE);
	pixels.show();
	motor(-1,_power_start);
	delay(_dur_start);
	motor(0,0);
	setPixel(PIXEL_MAIN, MY_LED_BLACK);
	pixels.show();

	the_clock->setTime(ID_TIME_LAST_START,time(NULL));

	pid_power = _power_high;
	flash_fxn = FUNCTION_NONE;
	last_change = millis();
}


void theClock::stopClock()
{
	init();
	motor(0,0);
	flash_fxn = FUNCTION_STOPPED;
	flash_count = 0;
	flash_time = millis();
	LOGU("stopClock()");
}



void theClock::onClockRunningChanged(const myIOTValue *desc, bool val)
{
	LOGU("onClockRunningChanged(%d)",val);
	if (val)
		startClock();
	else
		stopClock();
}


void theClock::onPIDModeChanged(const myIOTValue *desc, bool val)
{
	LOGU("onPIDModeChanged(%d)",val);
	pid_power = _power_high;
}


//===================================================================
// run()
//===================================================================

void theClock::run()
{
	//-------------------------------------------------
	// HALLS
	//-------------------------------------------------
	// read hall sensors through circular buffer
	// Seems to work better if we throw the first sample out

	for (int i=0; i<NUM_HALL_PINS; i++)
	{
		circ_buf[i][circ_ptr] = analogRead(hall_pins[i]) - hall_zero[i];
		int value = 0;
		for (int j=1; j<NUM_SAMPLES-1; j++)
		{
			value += circ_buf[i][j];
		}
		value = value / (NUM_SAMPLES-1);
		hall_value[i] = value;
	}

	circ_ptr++;
	if (circ_ptr >= NUM_SAMPLES)
		circ_ptr = 0;


	//-------------------------------------------------
	// POSITION
	//-------------------------------------------------
	// Determine incremental position  -4..-1 and 1..4
	//
	// Magnet to the left of a hall sensor is negative, to the right, positive.

	if (hall_value[0] < -_hall_thresh)	// detected the magnet to the left of sensor(0)
		position = -3;
	else if (hall_value[0] > _hall_thresh)
		position = -2;
	else if (hall_value[1] < -_hall_thresh)
		position = -1;

	// right side

	else if (hall_value[2] > _hall_thresh)
		position = 3;
	else if (hall_value[2] < -_hall_thresh)
		position = 2;
	else if (hall_value[1] > _hall_thresh)
		position = 1;

	// set the max_left and max_right

	if (position < 0 && position < max_left)
		max_left = position;
	if (position > 0 && position > max_right)
		max_right = position;


	//----------------------------------------------------------
	// CYCLE
	//----------------------------------------------------------
	// The full cycle is when the pendulum goes from positioni -1 to 1 and should take 1000 ms.
	// A half cycle is determined by the transition from position -1 to 1 in either direction.

	static int last_position = 0;
	static uint32_t motor_start = 0;
	static uint32_t motor_dur = 0;

	// if there have been no changes for 3 seconds
	// restart the clock

	uint32_t now = millis();
	if (clock_started && (now - last_change > 3000))
	{
		num_restarts++;
		LOGE("CLOCK STOPPED! - restarting!!");
		startClock();
		last_change = now;
	}

	if (last_position != position) //  && (now - last_change > 20))
	{
		int use_power = 0;
		last_change = now;

		// moving left to right

		if (last_position == -1 && position == 1)
		{
			num_beats++;

			if (clock_started && max_right < 3)
			{
				LOGE("STALL_RIGHT",0);
				num_stalls_right++;
			}

			if (cycle_start)	// not the first time through
			{
				cycle_duration = now - cycle_start;
				total_error += cycle_duration - 1000;

				if (total_error > high_error)
					high_error = total_error;
				if (total_error < low_error)
					low_error = total_error;
				if (cycle_duration < low_dur)
					low_dur = cycle_duration;
				if (cycle_duration > high_dur)
					high_dur = cycle_duration;

				if (_pid_mode && clock_started)
				{
					float this_p =  cycle_duration;		// this error
					this_p = this_p - 1000;
					float this_i = total_error;			// total error at this time
					float this_d = 0;

					if (prev_p)
					{
						this_d = this_p;
						this_d -= prev_p;
					}
					prev_p = this_p;

					this_p = this_p / 1000;
					this_i = this_i / 1000;
					this_d = this_d / 1000;

					int old_power = pid_power;
					float factor = 1 + (_pid_P * this_p) + (_pid_I * this_i) + (_pid_D * this_d);
					float new_power = pid_power * factor;
					if (new_power > _power_max) new_power = _power_max;
					if (new_power  < _power_low) new_power = _power_low;
					pid_power = new_power;

					if (pid_power < min_power_used)
						min_power_used = pid_power;
					if (pid_power > max_power_used)
						max_power_used = pid_power;

					motor_dur = _dur_right;
					use_power = pid_power;
				}

				// static mode right

				else if (clock_started)
				{
					motor_dur = _dur_right;
					if (max_right == 3)
						use_power = _power_low;
					else
						use_power = _power_high;
				}

				if (clock_started && !button_down && !flash_fxn)
				{
					setPixel(PIXEL_MAIN,
						total_error > 2000 ? MY_LED_BLUE :
						total_error < -2000 ? MY_LED_RED :
						total_error > 200 ? MY_LED_BLUECYAN :
						total_error < -200 ? MY_LED_ORANGE :
						cycle_duration < 995 ?  MY_LED_YELLOW :
						cycle_duration > 1005 ? MY_LED_CYAN :
						MY_LED_GREEN);
				}

				if (!_plot_values)
					LOGI("RIGHT(%d) dur(%d) power(%d) cycle(%d) error(%d)",max_right,motor_dur,use_power,cycle_duration,total_error);

				max_right = 0;

			}
			cycle_start = now;
		}

		// moving right to left

		if (clock_started && last_position == 1 && position == -1)
		{
			if (max_left > -3)
			{
				LOGE("STALL_LEFT",0);
				num_stalls_left++;
			}

			if (_dur_left)
			{
				motor_dur = _dur_left;
				if (_pid_mode)
					use_power = pid_power;
				else if (max_right == 3)
					use_power = _power_low;
				else
					use_power = _power_high;

				if (!_plot_values)
					LOGI(" LEFT(%d) dur(%d) power(%d)",max_left,motor_dur,use_power);
			}

			max_left = 0;
		}

		// IMPULSE

		if (use_power)
		{
			motor_start = now;
			motor(-1,use_power);
		}

		last_position = position;
	}


	//------------------------------------------------
	// stop the impulse
	//------------------------------------------------

	if (motor_start && now - motor_start > motor_dur)
	{
		motor_start = 0;
		motor_dur = 0;
		motor(0,0);
	}

	if (show_pixels)
		pixels.show();

	//------------------------
	// plot values
	//------------------------

	if (_plot_values == 1)
	{
		for (int i=0; i<NUM_HALL_PINS; i++)
		{
			Serial.print(hall_value[i]);
			Serial.print(",");
		}

		// Serial.print(cycle_duration);

		int err = cycle_duration - 1000;
		Serial.print(1000 + err * 20);
		Serial.print(",1000,-1400,1400,");
		Serial.println(position * 200);
	}

}	// theClock::run()



//===================================================================
// loop
//===================================================================

// virtual
void theClock::loop()	// override
{
	myIOTDevice::loop();

	//-----------------------------
	// handle button
	//-----------------------------

	uint32_t now = millis();
	if (now - button_chk > 33)	// 30 times per second
	{
		button_chk = now;
		static int last_press = 0;
		bool val = !digitalRead(PIN_BUTTON);
		if (button_down)
		{
			uint32_t dur = now - button_down;
			int press =
				dur > 8000 ? 3 :			// long press
				dur > 2000 ? 2 : 1;			// medium, short press
			int fxn =
				press == 3 ? FUNCTION_RESET :
				press == 2 ? getBool(ID_DEVICE_WIFI) ?
					FUNCTION_WIFI_OFF :
					FUNCTION_WIFI_ON :
				getBool(ID_RUNNING) ?
					FUNCTION_STOPPING :
					FUNCTION_STARTING;

			if (!val)
			{
				LOGD("button_up dur(%d) press(%d) fxn(%d)",dur,press,fxn);

				flash_fxn = fxn;
				button_down = 0;
				setPixel(PIXEL_MAIN,MY_LED_BLACK);
				pixels.show();
			}
			else if (last_press != press)
			{
				LOGD("button_down dur(%d) press(%d) fxn(%d)",dur,press,fxn);

				last_press = press;
				setPixel(PIXEL_MAIN,
					fxn == FUNCTION_RESET ? MY_LED_RED :
					fxn == FUNCTION_WIFI_OFF ? MY_LED_MAGENTA :
					fxn == FUNCTION_WIFI_ON ? MY_LED_BLUE :
					fxn == FUNCTION_STARTING ? MY_LED_GREEN :
					MY_LED_WHITE);
				pixels.show();
			}
		}
		else if (val && !button_down)
		{
			LOGD("button_down",0);
			button_down = now;
			flash_fxn = 0;
			flash_count = 0;
			flash_time = 0;
			last_press = 0;
		}
	}

	//------------------------
	// FLASH PIXELS
	//------------------------

	if (flash_fxn && now - flash_time > flash_dur)
	{
		uint32_t color;
		if (flash_on)
		{
			flash_on = 0;
			flash_dur = 400;
			flash_count++;
			color = MY_LED_BLACK;
		}
		else
		{
			flash_on = 1;
			flash_dur = 100;
			color =
				flash_fxn == FUNCTION_STARTING ? MY_LED_GREEN :
				flash_fxn == FUNCTION_WIFI_ON ? MY_LED_BLUE :
				flash_fxn == FUNCTION_WIFI_OFF ? MY_LED_MAGENTA :
				flash_fxn == FUNCTION_RESET ? MY_LED_RED :
				MY_LED_WHITE;	// STOPPED or STOPPING
				MY_LED_YELLOW;	// unknown function

		}
		flash_time = now;
		setPixel(PIXEL_MAIN,color);
		pixels.show();
		if (flash_count == 5)
		{
			int fxn = flash_fxn;
			flash_fxn = FUNCTION_NONE;

			if (fxn == FUNCTION_RESET)
				the_clock->factoryReset();
			else if (fxn == FUNCTION_WIFI_OFF)
				the_clock->setBool(ID_DEVICE_WIFI,0);
			else if (fxn == FUNCTION_WIFI_ON)
				the_clock->setBool(ID_DEVICE_WIFI,1);
			else if (fxn == FUNCTION_STARTING)
				the_clock->setBool(ID_RUNNING,1);
			else if (fxn == FUNCTION_STOPPING && clock_started)
			{
				the_clock->setBool(ID_RUNNING,0);
				flash_fxn = FUNCTION_STOPPED;
			}
			else if (fxn == FUNCTION_STOPPED)
				flash_fxn = FUNCTION_STOPPED;

			flash_dur = 5000;
			flash_count = 0;
		}
	}

	//---------------------------------
	// show stats every 60 beats
	//---------------------------------

	static uint32_t last_num_beats;

	if (_stat_interval &&
		num_beats % _stat_interval == 0 &&
		!_plot_values &&
		clock_started &&
		num_beats != last_num_beats)
	{
		last_num_beats = num_beats;

		setTime(ID_CUR_TIME,time(NULL));

		uint32_t secs = _cur_time - _time_last_start;
		uint32_t mins = secs / 60;
		uint32_t hours = mins / 60;
		uint32_t save_secs = secs;
		secs = secs - mins * 60;
		mins = mins - hours * 60;

		static char buf[80];
		sprintf(buf,"%02d:%02d:%02d  == %d secs",hours,mins,secs,save_secs);
		setString(ID_STAT_RUNTIME,buf);

		setInt(ID_STAT_BEATS,		num_beats);
		setInt(ID_STAT_RESTARTS,	num_restarts);
		setInt(ID_STAT_STALLS_L,	num_stalls_left);
		setInt(ID_STAT_STALLS_R,	num_stalls_right);
		setInt(ID_STAT_ERROR_L,		low_error);
		setInt(ID_STAT_ERROR_H,		high_error);
		setInt(ID_STAT_DUR_L,		low_dur);
		setInt(ID_STAT_DUR_H,		high_dur);
		setInt(ID_MIN_POWER_USED,	min_power_used);
		setInt(ID_MAX_POWER_USED,	max_power_used);
	}


	// Testing NTP
	// I am using the "pause" mode of _plot_values for this.
	// Typically I turn off the clock.
	// I am not sure if the ESP32 intercepts the UDP calls, but otherwise,
	//     there should be some drift between them over time.


	static uint32_t last_debug = 0;
	if (_plot_values == 2 && now > last_debug + 5000 )
	{
		last_debug = now;

		time_t sys_time = time(NULL);
		time_t ntp_time = getNtpTime();
		LOGD("---> sys_time=%d   ntp_time=%d",sys_time, ntp_time);
	}


}	// theClock::loop()



//------------------------------------------------------
// clockTask
//------------------------------------------------------
// disable the watchdog timer.
// the first param is number of seconds per check.
// at 30, I would get a WDT message, but the program
// kept running (as opposed to how it crashes normally
// on a WDT issue). Then I increased the delay to
// 0x0FFFFFFF (about 27 days).


#include <esp_task_wdt.h>

void theClock::clockTask(void *param)
{
	esp_task_wdt_init(0x0FFFFFFF, false);
    delay(1200);
    LOGI("starting clockTask loop on core(%d)",xPortGetCoreID());
    delay(1200);
    while (1)
    {
        vTaskDelay(0);		// 10 / portTICK_PERIOD_MS);
		uint32_t now = millis();
		static uint32_t last_sense = 0;
		if (now - last_sense > 2)		// why every 3 ms?  dunno.  it works for now
		{
			last_sense = now;
			the_clock->run();
		}
	}
}
