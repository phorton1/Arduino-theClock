#pragma once

#include <Arduino.h>
#include <myIOTDevice.h>

#define PIN_HALL1	32
#define PIN_HALL2	35
#define PIN_HALL3	34
#define PIN_HALL4	39
#define PIN_HALL5	36

#define PIN_ENA		27
#define PIN_INA1	25
#define PIN_INA2	26
#define PIN_ENB		12
#define PIN_INB1	14
#define PIN_INB2	33
#define PIN_ENC		4
#define PIN_INC1	17
#define PIN_INC2	5
#define PIN_END		16
#define PIN_IND1	21
#define PIN_IND2	23

#define PIN_LEDS	22
#define PIN_BUTTON  18

#define PIXEL_MODEL		0
#define PIXEL_MODER		1

#define PIXEL_DUR		2
#define PIXEL_ERROR     3

#define PIXEL_LEFT      4
#define PIXEL_RIGHT     5
#define NUM_PIXELS		6

#define MY_LED_BLACK    0x000000
#define MY_LED_RED      0x440000
#define MY_LED_GREEN    0x003300
#define MY_LED_BLUE     0x000044
#define MY_LED_CYAN     0x003333
#define MY_LED_YELLOW   0x333300
#define MY_LED_MAGENTA  0x330033
#define MY_LED_WHITE    0x444444



//------------------------
// theClock definition
//------------------------

#define ID_RUNNING			"RUNNING"
#define ID_PID_MODE			"PID_MODE"

#define ID_POWER_LOW      	"POWER_LOW"         // overrides PID controller if pendulum goes too far right (to far left uses zero!!)
#define ID_POWER_HIGH      	"POWER_HIGH"        // overriden by PID ...  Used with left and right durations in normal swing from -5 to 5
#define ID_POWER_SPRING     "POWER_SPRING"		// power for outside left magnets. Turned on to go faster when PID reaches MAX_POWER
#define ID_POWER_PULL     	"POWER_PULL"		// motor 3 - in STALLS only while going right, we try to "pull" the pendulum after the push of _dur_start/POWER_MAX

#define ID_DUR_LEFT      	"DUR_LEFT"          // normal left swing duration
#define ID_DUR_RIGHT      	"DUR_RIGHT"         // normal right swing duration
#define ID_DUR_START		"DUR_START"         // impulse given during start and emergency stalls
#define ID_DUR_PULL 		"DUR_PULL"          // starts immediately after dur start on STALLS; TURNS OFF on transition from position 4->5

#define ID_PID_P			"PID_P"         	// proportional value for PID
#define ID_PID_I			"PID_I"         	// integrated value for PID
#define ID_PID_D			"PID_D"         	// derivative value for PID

#define ID_CLEAR_STATS		"CLEAR_STATS"

#define ID_TIME_LAST_START  "LAST_START"
#define ID_STAT_TIME		"TIME_RUNNING"
#define ID_STAT_BEATS		"BEATS"
#define ID_STAT_RESTARTS	"RESTARTS"
#define ID_STAT_STALLS_L	"STALLS_LEFT"
#define ID_STAT_STALLS_R	"STALLS_RIGHT"
#define ID_STAT_OVER_L		"OVER_LEFT"
#define ID_STAT_OVER_R		"OVER_RIGHT"
#define ID_STAT_ERROR_L		"ERROR_LOW"
#define ID_STAT_ERROR_H		"ERROR_HIGH"



class theClock : public myIOTDevice
{
public:

    theClock();
    ~theClock() {}

    virtual void setup() override;
	virtual void loop() override;

private:

    static const valDescriptor m_clock_values[];

	static bool _clock_running;
	static bool _pid_mode;

	static int _power_low;		// used when reaches extremes; bypasses PID
	static int _power_high;		// used when reaches normal and !PID (pid takes it over)
	static int _power_spring;	// motor 0 - if power max in PID mode (go faster!) - it stays on
	static int _power_pull;		// motor 3 - in STALLS only while going right, we try to "pull" the pendulum after the push of _dur_start/POWER_MAX

	static int _dur_left;		// left duration for normal and extremes
	static int _dur_right;		// right duration for normal and extremes
	static int _dur_start;		// duration during startup and stalls; uses POWER_MAX and _power_pull
	static int _dur_pull;		// starts immediately after dur start on STALLS

	static float _pid_P;
	static float _pid_I;
	static float _pid_D;

    static uint32_t _time_last_start;
	static String _stat_time;
	static uint32_t _stat_beats;
	static uint32_t _stat_restarts;
	static uint32_t _stat_stalls_left;
	static uint32_t _stat_stalls_right;
	static uint32_t _stat_over_left;
	static uint32_t _stat_over_right;
	static int _stat_error_low;
	static int _stat_error_high;

	static void startClock();
	static void stopClock();
	static void clearStats();

    static void onClockRunningChanged(const myIOTValue *desc, bool val);
    static void onPIDModeChanged(const myIOTValue *desc, bool val);

	static void run();
	static void clockTask(void *param);
};


extern theClock *the_clock;