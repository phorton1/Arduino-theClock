
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
	//
	// So *maybe* I can uses the myIOT architecture!
	//
	// 2022-11-25  11:20a
	//
	// 		Clock seems to be working well.  However, the arduino "loop()" seems to
	// 		be blocking.  The pixels used to be shown there, and they froze, as well
	// 		as the web UI.  I moved the pixels.show() to theClock::run() but don't really
	// 		understand the problem.
	//
	// 		In 10 hour run started at 10:21A, so far, NO right stalls! and
	// 		   max error_low=-298, high=142.  Telnet worked while Web/Pixels
	// 		 were blocked.
	//
	// 		Recompiled, retarted, re-synced.
	//
	// 11:40 fuck.  It reboot "bad heap"
	//
	//		2022-11-25  11:39:17 140:107   dur=997   error=-142  power=180
	//		2022-11-25  11:39:17 140:107       power=180 myPID(-0.002000,-0.142000,-0.007000) factor=0.968800  new=180.000000
	//		2022-11-25  11:39:18 140:107   dur=1007  error=-135  power=180
	//		2022-11-25  11:39:18 140:107       power=180 myPID(0.008000,-0.135000,-4294967.500000) factor=0.984200  new=180.000000
	//		2022-11-25  11:39:19 143:107   dur=997   error=-138  power=180
	//		2022-11-25  11:39:19 143:107       power=180 myPID(-0.002000,-0.138000,-0.010000) factor=0.969600  new=180.000000
	//		2022-11-25  11:39:20 143:107   dur=1006  error=-132  power=180
	//		2022-11-25  11:39:20 143:107       power=180 myPID(0.007000,-0.132000,-4294967.500000) factor=0.983400  new=180.000000
	//		2022-11-25  11:39:20 144:107       WS[0] Disconnected!
	//		2022-11-25  11:39:20 147:107       WS[1] Disconnected!
	//		CORRUPT HEAP: Bad head at 0x3ffd11a8. Expected 0xabba1234 got 0x3ffd6740
	//		abort() was called at PC 0x40086ff9 on core 1
	//
	//		ELF file SHA256: 0000000000000000
	//
	//		Backtrace: 0x40088d98:0x3ffd2d10 0x40089015:0x3ffd2d30 0x40086ff9:0x3ffd2d50 0x40087125:0x3ffd2d80 0x40109fc7:0x3ffd2da0 0x40104ba9:0x3ffd3060 0x40104a9d:0x3ffd30b0 0x4008e1ed:0x3ffd30e0 0x40081e76:0x3ffd3100 0x40086ef1:0x3ffd3120 0x4000bec7:0x3ffd3140 0x40162ff9:0x3ffd3160 0x400ddfd9:0x3ffd3180 0x40169a19:0x3ffd31a0 0x400dcc5a:0x3ffd31c0 0x401699db:0x3ffd31e0 0x400dd57a:0x3ffd3200 0x400dd65a:0x3ffd3230 0x400dd676:0x3ffd3250 0x400d9081:0x3ffd3270 0x4008aada:0x3ffd3290
	//
	//		Rebooting...
	//		ets Jun  8 2016 00:22:57
	//
	// 11:45a = I think it's time for a checkin, plus the fusion prhParam thing





//----------------------------
// halls
//----------------------------

#define HALL_THRESH		30

#define NUM_HALL_PINS	5
#define NUM_SAMPLES     5

const int hall_pins[NUM_HALL_PINS] =
{
	PIN_HALL1,
	PIN_HALL2,
	PIN_HALL3,
	PIN_HALL4,
	PIN_HALL5,
};

static int circ_ptr = 0;
static int circ_buf[NUM_HALL_PINS][NUM_SAMPLES];
static int hall_value[NUM_HALL_PINS];
static int hall_zero[NUM_HALL_PINS] = {1869,1804,1839,1781,1840};


//----------------------------
// motors
//----------------------------

#define NUM_MOTORS		4

#define POWER_MAX   255

#define PWM_FREQUENCY	5000
#define PWM_RESOLUTION	8

int pin_enable[NUM_MOTORS] = {
	PIN_ENA,
	PIN_ENB,
	PIN_ENC,
	PIN_END,
};
int pin_in1[NUM_MOTORS] = {
	PIN_INA1,
	PIN_INB1,
	PIN_INC1,
	PIN_IND1,
};
int pin_in2[NUM_MOTORS] = {
	PIN_INA2,
	PIN_INB2,
	PIN_INC2,
	PIN_IND2,
};


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

static bool plot_values   	= 0;

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
static int num_over_right  = 0;
static int32_t high_error   = 0;
static int32_t low_error    = 0;



#if 0
	void one_time_calibrate_hall()
	{
		// assuming pendulum is hanging down
		// set zero for sensors 1,2,4, and 5

		display(0,"calibrating hall pins - pendulum must be stopped, center",0);
		delay(500);
		for (int i=0; i<NUM_HALL_PINS; i++)
		{
			if (i != 2)
				hall_zero[i] = analogRead(hall_pins[i]);
		}
		delay(500);

		// then set the central sensor

		display(0,"move pendulum to one side and press any key",0);
		while (!Serial.available()) { delay(5); }
		int c = Serial.read();
		delay(100);
		hall_zero[2] = analogRead(hall_pins[2]);
		delay(500);
		display(0,"hall calibration complete %d,%d,%d,%d,%d",
			hall_zero[0],
			hall_zero[1],
			hall_zero[2],
			hall_zero[3],
			hall_zero[4]);
	}
#endif


// static IRAM_ATTR uint32_t int_count = 0;
// static IRAM_ATTR void testInterrupt()
// {
// 	int_count++;
// }



// virtual
void theClock::setup()	// override
{
	LOGU("theClock::setup() started");

	myIOTDevice::setup();

	pixels.setPixelColor(0,MY_LED_RED);
	pixels.show();
	delay(500);

	for (int i=0; i<NUM_HALL_PINS; i++)
	{
		pinMode(hall_pins[i],INPUT);
	}

	// attachInterrupt(hall_pins[3], testInterrupt, FALLING);

	for (int i=0; i<NUM_MOTORS; i++)
	{
		// the ESP32 PWM channel used IS the 'num' of the pin

		ledcSetup(i, PWM_FREQUENCY, PWM_RESOLUTION);
		ledcAttachPin(pin_enable[i], i);
		ledcWrite(i,0);
		pinMode(pin_in1[i],OUTPUT);
		pinMode(pin_in2[i],OUTPUT);
		digitalWrite(pin_in1[i],0);
		digitalWrite(pin_in2[i],0);
	}

	pixels.setPixelColor(0,MY_LED_GREEN);
	pixels.show();
	delay(500);
	pixels.setPixelColor(0,MY_LED_BLACK);
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

}	// theClock::setup()



void motor(int num, int state, int power)
{
	int use_power = state ? power : 0;
	ledcWrite(num, use_power);
	digitalWrite(pin_in1[num],state == 1  ? 1 : 0);
	digitalWrite(pin_in2[num],state == -1 ? 1 : 0);
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
	num_restarts    = 0;
	num_stalls_left  = 0;
	num_stalls_right = 0;
	num_over_left = 0;
	num_over_right = 0;
	high_error   = 0;
	low_error    = 0;
}


void theClock::clearStats()
{
	LOGU("STATISTICS CLEARED");

	total_error = 0;
	prev_p = 0;

	num_beats = 0;
	num_restarts    = 0;
	num_stalls_left  = 0;
	num_stalls_right = 0;
	num_over_left = 0;
	num_over_right = 0;
	high_error   = 0;
	low_error    = 0;

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
	motor(0,0,0);
	motor(1,0,0);
	motor(2,0,0);
	motor(3,0,0);
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
	// Determine incremental position  -6..-1 and 1..6
	//
	// Magnet to the left of a hall sensor is negative, to the right, positive.
	//
	// With the magic threshold of 30, there is a small gap between the sensors,
	// so this just happens to invariantly gives us 10 'static' positions, -5..-1 and 1..5.
	// However we work from the "outside in" just in case there is overlap,
	// in order to get good zero crossings ... especially the critical -1..1 central crossing.
	//
	// We get the final two extreme positions, -6, and 6, by noting that the outside sensor
	// goes to zero, but does not cross, if the magnet is sufficiently far outside from it.
	// We keep state variables to detect this case, so that when it returns we can set the
	// 'six' positions -6 and 6.

	static int l_six = 0;
	static int r_six = 0;
		// These are state variables.
		//      0 = an "inside" position was encountered, so we cleared the state
		//      1 = we are outside of the sensor for the first time, set the nominal position
		//      2 = sensor went to zero after being in state 1

	if (l_six == 1 && abs(hall_value[0]) < HALL_THRESH)
		l_six = 2;		// left sensor was outside and went to zero

	if (r_six == 1 && abs(hall_value[4] < HALL_THRESH))
		r_six = 2;		// right sensor was outside and went to zero

	if (hall_value[0] < -HALL_THRESH)	// detected the magnet to the left of sensor(0)
	{
		if (l_six == 2)					// and it previouly was to the left, but had gone to zero
		{
			position = -6;				// so therefore we WERE outside of it, and set the extreme position
		}
		else							// first times through
		{
			l_six = 1;
			position = -5;
		}
	}
	else if (hall_value[0] > HALL_THRESH)
		position = -4;
	else if (hall_value[1] < -HALL_THRESH)
		position = -3;
	else if (hall_value[1] > HALL_THRESH)
		position = -2;
	else if (hall_value[2] < -HALL_THRESH)
		position = -1;

	// main threshold crossing

	else if (hall_value[4] > HALL_THRESH)
	{
		if (r_six == 2)
		{
			position = 6;
		}
		else
		{
			r_six = 1;
			position = 5;
		}
	}
	else if (hall_value[4] < -HALL_THRESH)
		position = 4;
	else if (hall_value[3] > HALL_THRESH)
		position = 3;
	else if (hall_value[3] < -HALL_THRESH)
		position = 2;
	else if (hall_value[2] > HALL_THRESH)
		position = 1;

	// reset 'six' state variables on any move to interior position

	if (position > -5)
		l_six = 0;
	if (position < 5)
		r_six = 0;

	// set maximum points reached during cycle
	// and show diagnostic pixels

	static int save_left = 0;
	static int save_right = 0;
	if (position < 0 && position < max_left)
	{
		max_left = position;

		if (max_left == -6)
			setPixel(PIXEL_LEFT,MY_LED_MAGENTA);
		else if (max_left == -5)
			setPixel(PIXEL_LEFT,MY_LED_GREEN);
		else
			setPixel(PIXEL_LEFT,MY_LED_RED);
	}
	if (position > 0 && position > max_right)
	{
		max_right = position;

		if (max_right == 6)
			setPixel(PIXEL_RIGHT,MY_LED_MAGENTA);
		else if (max_right == 5)
			setPixel(PIXEL_RIGHT,MY_LED_GREEN);
		else
			setPixel(PIXEL_RIGHT,MY_LED_RED);
	}


	//----------------------------------------------------------
	// CYCLE
	//----------------------------------------------------------
	// The full cycle is when the pendulum goes from positioni -1 to 1 and should take 1000 ms.
	// A half cycle is determined by the transition from position -1 to 1 in either direction.

	static int last_position = 0;
	static uint32_t motor_start = 0;
	static uint32_t motor_dur = 0;
	static uint32_t pull_start = 0;
	static bool pull_next = 0;

	static uint32_t last_change = 0;

	// if there have been no changes for 1.5 seconds
	// give a small impulse to get the pendulum moving

	uint32_t now = millis();
	if (clock_started && (now - last_change > 1500))
	{
		last_change = now;
		num_restarts++;
		LOGE("CLOCK STOPPED! - restarting!!");
		motor(2,-1,POWER_MAX);
		delay(30);
	}

	if (last_position != position)
	{
		last_change = now;

		// MAIN HALF CYCLE

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

			if (!plot_values)
				LOGU("dur=%-4d  error=%-4d  power=%d",cycle_duration,total_error,_power_high);

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

			setPixel(PIXEL_RIGHT,MY_LED_BLACK);

			// on any left to right, for a short while
			// we repulse with motor 1

			if (clock_started && max_right <= 5)
				motor(1,-1,POWER_MAX);

			save_right = max_right;
			max_right = 0;

			if (_pid_mode)
			{
				float this_p =  cycle_duration;		// this error
				this_p = this_p - 999; // 1000;
				float this_i = total_error;		// total error at this time
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

				LOGI("power=%d myPID(%f,%f,%f) factor=%f  new=%f",_power_high,this_p,this_i,this_d,factor,new_power);

				_power_high = new_power;

				if (_power_spring && _power_high == POWER_MAX)
				{
					motor(0,-1,_power_spring);
				}
				else
					motor(0,0,0);

			}	// pid_mode
		}	// main cycle

		// right to left HALF CYCLE

		else if (last_position == 1 && position == -1)
		{
			setPixel(PIXEL_LEFT,MY_LED_BLACK);
			save_left = max_left;
			max_left = 0;
		}

		//-------------------------------------------------------------
		// POWER IMPULSE
		//-------------------------------------------------------------
		// the transitions from 1 to 2 and -1 to -2 are when we put
		// an impulse into the pendulum.  This section contains some
		// troubling heuristics.  Written before the PID implementation,
		// these lines attempt to get the pendulum moving so as to just
		// cross the outermost hall sensors 0 and 4, which correspond
		// to positions -5 and 5.  If it didn't make it to the hall sensor,
		// we consider that an emergency and use the maximum power,
		// and a shorter pulse (dur_start) which is intended to get it
		// moving from very short swings until it starts crossing the
		// hall sensor again.
		//
		// We receive copies of the max left and right positions via 'save_left'
		// and 'save_right' so that the globals can be reset for the next cycle
		// at the main center zero crossing (which already happend).

		if (clock_started)
		{
			if (last_position == -1 && position == -2)	// moving right to left
			{
				// display(0,"right save=%d max=%d",save_right,max_right);

				int use_power = 0;
				int use_dur = 0;

				// while moving left
				// the global 'max_right' is correct at this time, not save_right!
				// we look at the right with more interest as that is where it has
				// to 'push' the pendulum, and takes more work.  So this case is
				// added that if the right doesn't make 5, we put left in 'emergency'
				// mode too.

				if (max_right < 5)		// emergency!! right is stalled!
				{
					use_power = POWER_MAX;
					use_dur = _dur_start;
					setPixel(PIXEL_MODEL,MY_LED_MAGENTA);
				}
                //
				// // on the other hand, if we have gone too far left,
				// // we just plain DONT put any energy into the slack part of
				// // the pendulum swing (to the left requies almost no energy)

				else if (save_left == -6)
				{
					use_power = _power_low;	//	0; 	// _power_low;
					use_dur = _dur_left;	// 0; 	// max_right >= 5 ? 0 : _dur_left;
					setPixel(PIXEL_MODEL,MY_LED_BLUE);
					num_over_left++;
					LOGE("OVER LEFT",0);
				}

				// normal case if left made it to the sensor
				// power_high will be controlled by the PID
				// with the normal left duration

				else if (save_left == -5)
				{
					use_power = _power_high;
					use_dur = _dur_left;
					setPixel(PIXEL_MODEL,MY_LED_GREEN);
				}

				// emergency - left is stalled!
				// give it max power and use dur_start.

				else
				{
					LOGE("LEFT STALLED",0);
					use_power = POWER_MAX;
					use_dur = _dur_start;
					setPixel(PIXEL_MODEL,MY_LED_RED);
					num_stalls_left++;
				}

				save_left = 0;
				motor_start = now;
				motor_dur = use_dur;
				motor(1,-1,use_power);
			}
			else if (last_position == 1 && position == 2)	// moving left to right
			{
				int use_power = 0;
				int use_dur = 0;

				motor(1,0,0);		// turn off short impulse for left to right motion

				// right went too far. use power_low
				// this bypasses pid controller by using power_low

				if (save_right == 6)
				{
					use_power = _power_low;
					use_dur = _dur_right;
					setPixel(PIXEL_MODER,MY_LED_BLUE);
					num_over_right++;
					LOGE("OVER RIGHT",0);
				}

				// normal, use power_high (PID) and constant dur_right

				else if (save_right == 5)
				{
					use_power = _power_high;
					use_dur = _dur_right;
					setPixel(PIXEL_MODER,MY_LED_GREEN);
				}

				// emergency - right stalled!
				// use max power and dur_start

				else
				{
					num_stalls_right++;
					use_power = POWER_MAX;
					use_dur = _dur_start;
					setPixel(PIXEL_MODER,MY_LED_RED);
					const char *msg_pulling = "";
					if (_dur_pull && _power_pull)
					{
						pull_next = true;
						msg_pulling = "!PULLING!";
					}
					LOGE("RIGHT STALLED %s",msg_pulling);
				}

				save_right = 0;
				motor_start = now;
				motor_dur = use_dur;
				motor(2,-1,use_power);
			}

			// turn off pull if we made it to position 5

			else if (pull_start && position == 5)
			{
				pull_start = 0;
				motor(3,0,0);
				setPixel(PIXEL_MODEL,MY_LED_BLACK);
			}

		}	// clock_started

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
		motor(1,0,0);
		motor(2,0,0);

		setPixel(PIXEL_MODEL,MY_LED_BLACK);

		if (pull_next)
		{
			pull_next = 0;
			motor(3,1,_power_pull);
			setPixel(PIXEL_MODER,MY_LED_YELLOW);
			pull_start = now;
		}
		else
		{
			setPixel(PIXEL_MODER,MY_LED_BLACK);
		}
	}

	if (pull_start && now - pull_start > _dur_pull)
	{
		pull_start = 0;
		motor(3,0,0);
		setPixel(PIXEL_MODEL,MY_LED_BLACK);
	}


	if (show_pixels)
		pixels.show();


	if (plot_values)
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

	if (!plot_values &&
		clock_started &&
		num_beats % 10 == 0 &&
		num_beats != last_num_beats)
	{
		last_num_beats = num_beats;

		uint32_t secs = time(NULL) - _time_last_start;
		uint32_t mins = secs / 60;
		uint32_t hours = mins / 60;
		uint32_t save_secs = secs;
		secs = secs - mins * 60;
		mins = mins - hours * 60;

		static char buf[80];
		sprintf(buf,"%02d:%02d:%02d  == %d secs",hours,mins,secs,save_secs);
		setString(ID_STAT_TIME,buf);

		setInt(ID_STAT_BEATS,		num_beats);
		setInt(ID_STAT_RESTARTS,	num_restarts);
		setInt(ID_STAT_STALLS_L,	num_stalls_left);
		setInt(ID_STAT_STALLS_R,	num_stalls_right);
		setInt(ID_STAT_OVER_L,		num_over_left);
		setInt(ID_STAT_OVER_R,		num_over_right);
		setInt(ID_STAT_ERROR_L,		low_error);
		setInt(ID_STAT_ERROR_H,		high_error);
	}

	//	static uint32_t last_int_count = 0;
	//	if (last_int_count != int_count)
	//	{
	//		last_int_count = int_count;
	//		LOGU("----> INT_COUNT(%d)",last_int_count);
	//	}

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
