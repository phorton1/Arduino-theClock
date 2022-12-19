
#include "theClock.h"
#include <myIOTLog.h>
#include <Adafruit_NeoPixel.h>



#define CLOCK_USE_TASK	1
	// there is too much critical timing in the clock run() method to
	// allow it to run on the same core as the webservers, etc.  When just run from
	// the default core(1), along with the Webserver, it regularly misses crossings,
	// etc, esp when the myIOT WebSocket goes on and off.
	//
	// So I made it a task on the unused core 0. However, tasks are limited to 10ms
	// time slices, and so it did not work at first.
	//
	// I was able to get it working. First I changed the call to vTaskDelay(0)
	// as I read that would yield without delay.  No joy.  I set the priority
	// of the task down to 1, no joy.  Finally I disabled the Watch Dog Timer,
	// (see code below) and that seems to work.


//----------------------------
// halls
//----------------------------

#define HALL_THRESH		30

#define NUM_HALL_PINS	3
#define NUM_SAMPLES     5

const int hall_pins[NUM_HALL_PINS] =
{
	PIN_HALL1,
	PIN_HALL2,
	PIN_HALL3,
	// PIN_HALL4,
	// PIN_HALL5,
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

#define POWER_MAX   255

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

// PID

static int32_t total_error = 0;
static uint32_t prev_p = 0;
static int save_high_power = 0;

// Statistics

static uint32_t num_beats   = 0;
static int num_restarts     = 0;
static int num_stalls_left  = 0;
static int num_stalls_right = 0;
static int num_over_left 	= 0;
static int num_over_right   = 0;
static int32_t low_error    = 0;
static int32_t high_error   = 0;
static int32_t low_dur      = 0;
static int32_t high_dur     = 0;




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

	pixels.setPixelColor(0,MY_LED_YELLOW);
	pixels.show();
	myIOTDevice::setup();

	pixels.setPixelColor(0,MY_LED_GREEN);
	pixels.show();

	save_high_power = _power_high;
	#if CLOCK_USE_TASK
	    LOGI("starting clockTask");
		xTaskCreatePinnedToCore(clockTask,
			"clockTask",
			8192,           // task stack
			NULL,           // param
			1,  	        // note that the priority is higher than one
			NULL,           // returned task handle
			ESP32_CORE_OTHER);
	#endif

	if (_clock_running)
		startClock();

	LOGU("theClock::setup() finished");
	pixels.setPixelColor(0,MY_LED_BLACK);
	pixels.show();

}	// theClock::setup()



void motor(int state, int power)
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
	num_over_left = 0;
	num_over_right = 0;
	low_error = 0;
	high_error = 0;
	low_dur = 32767;
	high_dur = 0;
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
	num_over_left = 0;
	num_over_right = 0;
	low_error = 0;
	high_error = 0;
	low_dur = 32767;
	high_dur = 0;

	// resetting the statistics also resets the
	// start time for display.

	if (clock_started)
		the_clock->setTime(ID_TIME_LAST_START,time(NULL));

}

void theClock::startClock()
{
	init();
	clock_started = 1;
	the_clock->setTime(ID_TIME_LAST_START,time(NULL));
	LOGU("startClock()");
}


void theClock::stopClock()
{
	init();
	motor(0,0);
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
	if (val)
	{
		save_high_power = _power_high;
	}
	else
	{
		 _power_high = save_high_power;
	}
}



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
	//
	// We get the final two extreme positions, -4, and 4, by noting that the outside sensor
	// goes to zero, but does not cross, if the magnet is sufficiently far outside from it.
	// We keep state variables to detect this case, so that when it returns we can set the
	// 'past' positions -4 and 4.

		static int past_l = 0;
		static int past_r = 0;
			// These are state variables.
			//      0 = an "inside" position was encountered, so we cleared the state
			//      1 = we are outside of the sensor for the first time, set the nominal position
			//      2 = sensor went to zero after being in state 1

		if (past_l == 1 && abs(hall_value[0]) < HALL_THRESH)
			past_l = 2;		// left sensor was outside and went to zero

		if (past_r == 1 && abs(hall_value[2] < HALL_THRESH))
			past_r = 2;		// right sensor was outside and went to zero

		if (hall_value[0] < -HALL_THRESH)	// detected the magnet to the left of sensor(0)
		{
			if (past_l == 2)				// and it previouly was to the left, but had gone to zero
			{
				position = -4;				// so therefore we WERE outside of it, and set the extreme position
			}
			else							// first times through
			{
				past_l = 1;
				position = -3;
			}
		}
		else if (hall_value[0] > HALL_THRESH)
			position = -2;
		else if (hall_value[1] < -HALL_THRESH)
			position = -1;

		// right side

		else if (hall_value[2] > HALL_THRESH)
		{
			if (past_r == 2)
			{
				position = 4;
			}
			else
			{
				past_r = 1;
				position = 3;
			}
		}
		else if (hall_value[2] < -HALL_THRESH)
			position = 2;
		else if (hall_value[1] > HALL_THRESH)
			position = 1;

		// reset 'past' state variables on any move to interior position

		if (position >= -2)
			past_l = 0;
		if (position <= 2)
			past_r = 0;

	// set maximum points reached during cycle
	// and show diagnostic pixels

	static int save_left = 0;
	static int save_right = 0;
	if (position < 0 && position < max_left)
	{
		max_left = position;

		#if WITH_DIAG_PIXELS
			if (max_left == -4)
				setPixel(PIXEL_LEFT,MY_LED_MAGENTA);
			else if (max_left == -3)
				setPixel(PIXEL_LEFT,MY_LED_GREEN);
			else
				setPixel(PIXEL_LEFT,MY_LED_RED);
		#endif
	}
	if (position > 0 && position > max_right)
	{
		max_right = position;

		#if WITH_DIAG_PIXELS
			if (max_right == 4)
				setPixel(PIXEL_RIGHT,MY_LED_MAGENTA);
			else if (max_right == 3)
				setPixel(PIXEL_RIGHT,MY_LED_GREEN);
			else
				setPixel(PIXEL_RIGHT,MY_LED_RED);
		#endif
	}


	//----------------------------------------------------------
	// CYCLE
	//----------------------------------------------------------
	// The full cycle is when the pendulum goes from positioni -1 to 1 and should take 1000 ms.
	// A half cycle is determined by the transition from position -1 to 1 in either direction.

	static int last_position = 0;
	static uint32_t motor_start = 0;
	static uint32_t motor_dur = 0;
	static uint32_t last_change = 0;

	// if there have been no changes for 1.5 seconds
	// give a small impulse to get the pendulum moving

	uint32_t now = millis();
	if (clock_started && (now - last_change > 1500))
	{
		last_change = now;
		num_restarts++;
		LOGE("CLOCK STOPPED! - restarting!!");
		#if WITH_DIAG_PIXELS == 0
			setPixel(PIXEL_MAIN, MY_LED_WHITE);
		#endif
		motor(-1,POWER_MAX);
		delay(200);
		motor(0,0);
	}

	if (last_position != position)
	{
		last_change = now;

		// MAIN HALF CYCLE (moving right to left)

		if (last_position == -1 && position == 1)	// moving left to right
		{
			num_beats++;

			if (cycle_start)	// not the first time through
			{
				cycle_duration = now - cycle_start;
				total_error += cycle_duration - 1000;
			}
			cycle_start = now;

			if (total_error > high_error)
				high_error = total_error;
			if (total_error < low_error)
				low_error = total_error;
			if (cycle_duration < low_dur)
				low_dur = cycle_duration;
			if (cycle_duration > high_dur)
				high_dur = cycle_duration;

			if (!_plot_values)
				LOGU("dur=%-4d  error=%-4d  power=%d",cycle_duration,total_error,_power_high);

			#if WITH_DIAG_PIXELS
				setPixel(PIXEL_DUR,
					cycle_duration < 998 ? MY_LED_RED :
					cycle_duration > 1001 ? MY_LED_BLUE :
					MY_LED_GREEN);
				setPixel(PIXEL_ERROR,
					total_error < -2000 ? MY_LED_RED :
					total_error < -200 ? MY_LED_MAGENTA :
					total_error > 2000 ? MY_LED_BLUE :
					total_error > 200 ? MY_LED_CYAN :
					MY_LED_GREEN);
			#else
				setPixel(PIXEL_MAIN,
					total_error > 2000 ? MY_LED_BLUE :
					total_error < -2000 ? MY_LED_RED :
					total_error > 200 ? MY_LED_BLUECYAN :
					total_error < -200 ? MY_LED_ORANGE :
					cycle_duration < 995 ?  MY_LED_YELLOW :
					cycle_duration > 1005 ? MY_LED_CYAN :
					MY_LED_GREEN);
			#endif

			if (_pid_mode)
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

				float factor = 1 + (_pid_P * this_p) + (_pid_I * this_i) + (_pid_D * this_d);
				float new_power = _power_high * factor;
				if (new_power > POWER_MAX) new_power = POWER_MAX;
				if (new_power  < _power_low) new_power = _power_low;

				if (!_plot_values)
					LOGI("power=%d myPID(%f,%f,%f) factor=%f  new=%f",_power_high,this_p,this_i,this_d,factor,new_power);
				_power_high = new_power;

			}	// pid_mode

			// RIGHT IMPULSE

			if (clock_started)
			{
				int use_power = 0;
				int use_dur = 0;

				// if (_pid_mode)
				// {
				// 	use_power = _power_high;
				// 	use_dur = _dur_right;
				// }
				// else

				if (max_right > 3)			// right went too far. use power_low (bypasses pid controller using power_low)
				{
					use_power = _power_low;
					use_dur = _dur_right;
					#if WITH_DIAG_PIXELS
						setPixel(PIXEL_MODER,MY_LED_BLUE);
					#endif
					num_over_right++;
				}
				else if (max_right == 3)	// normal, use power_high (PID) and constant dur_right
				{
					use_power = _power_high;
					use_dur = _dur_right;
					#if WITH_DIAG_PIXELS
						setPixel(PIXEL_MODER,MY_LED_GREEN);
					#endif
				}
				else						// emergency - right stalled!
				{
					use_power = POWER_MAX;
					use_dur = max_right == 2 ? _dur_stall : _dur_start;
					#if WITH_DIAG_PIXELS
						setPixel(PIXEL_MODER,MY_LED_RED);
					#endif
					LOGE("STALL_RIGHT",0);
					num_stalls_right++;
				}

				motor_start = now;
				motor_dur = use_dur;
				motor(-1,use_power);

			}	// clock_running (RIGHT IMPULSE)

			max_right = 0;

		}	// main cycle

		// right to left HALF CYCLE

		else if (last_position == 1 && position == -1)
		{
			if (clock_started)
			{
				int use_power = 0;
				int use_dur = 0;

				// if (_pid_mode)
				// {
				// 	use_power = _power_high;
				// 	use_dur = _dur_left;
				// }
				// else

				if (max_left < -3)			// left went to far, NO PULSE
				{
					use_power = 0;
					use_dur = 0;
					#if WITH_DIAG_PIXELS
						setPixel(PIXEL_MODEL,MY_LED_BLUE);
					#endif
					num_over_left++;
				}

				// normal case if left made it to the sensor
				// power_high will be controlled by the PID
				// with the normal left duration

				else if (max_left == -3)
				{
					use_power = _power_high;
					use_dur = _dur_left;
					#if WITH_DIAG_PIXELS
						setPixel(PIXEL_MODEL,MY_LED_GREEN);
					#endif
				}

				// emergency - left is stalled!

				else
				{
					use_power = POWER_MAX;
					use_dur = max_left == -2 ? _dur_stall : _dur_start;
					#if WITH_DIAG_PIXELS
						setPixel(PIXEL_MODEL,MY_LED_RED);
					#endif
					LOGE("STALL_LEFT",0);
					num_stalls_left++;
				}

				motor_start = now;
				motor_dur = use_dur;
				motor(-1,use_power);

			}	// LEFT IMPULSE

			max_left = 0;

		}	// right to left HALF CYCLE

		last_position = position;

	}	// position changed


	//------------------------------------------------
	// FINISH UP
	//------------------------------------------------
	// stop the impulse, plot values, show leds, etc

	if (motor_start && now - motor_start > motor_dur)
	{
		motor_start = 0;
		motor_dur = 0;
		motor(0,0);
		#if WITH_DIAG_PIXELS
			setPixel(PIXEL_MODEL,MY_LED_BLACK);
			setPixel(PIXEL_MODER,MY_LED_BLACK);
		#endif
	}

	if (show_pixels)
		pixels.show();

	if (_plot_values == 1)
	{
		for (int i=0; i<NUM_HALL_PINS; i++)
		{
			Serial.print(hall_value[i]);
			Serial.print(",");
		}
		Serial.print("-1400,1400,");
		Serial.println(position * 200);
	}

}	// theClock::run()



// virtual
void theClock::loop()	// override
{
	myIOTDevice::loop();

	#if !CLOCK_USE_TASK
		uint32_t now = millis();
		static uint32_t last_sense = 0;
		if (now - last_sense > 2)		// why every 3 ms?  dunno.  it works for now
		{
			last_sense = now;
			the_clock->run();
		}
	#endif

	// show stats every 10 beats

	static uint32_t last_num_beats;

	if (!_plot_values &&
		clock_started &&
		num_beats % 10 == 0 &&
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
		setInt(ID_STAT_OVER_L,		num_over_left);
		setInt(ID_STAT_OVER_R,		num_over_right);
		setInt(ID_STAT_ERROR_L,		low_error);
		setInt(ID_STAT_ERROR_H,		high_error);
		setInt(ID_STAT_DUR_L,		low_dur);
		setInt(ID_STAT_DUR_H,		high_dur);
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
