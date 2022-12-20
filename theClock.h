#pragma once

#include <Arduino.h>
#include <myIOTDevice.h>

#define COMPILE_VERSION  2
	// sets the (old) static HALL calibration values for V1 vs V2
	// and the myIOT device name and version

#define PIN_HALL1	32
#define PIN_HALL2	35
#define PIN_HALL3	34

// Note that ENB, INB1, and INB2 are sent out for backward
// compatability for V1 circuit board but unused on V2

#define PIN_ENA		27
#define PIN_INA1	25
#define PIN_INA2	26
#define PIN_ENB		4
#define PIN_INB1	17
#define PIN_INB2	5

#define PIN_LEDS	22
#define PIN_BUTTON  18

#define PIXEL_MAIN		0
#define NUM_PIXELS		1

#define MY_LED_BLACK    0x000000
#define MY_LED_RED      0x440000
#define MY_LED_GREEN    0x003300
#define MY_LED_BLUE     0x000044
#define MY_LED_CYAN     0x003333
#define MY_LED_YELLOW   0x333300
#define MY_LED_MAGENTA  0x330033

#define MY_LED_WHITE    0x444444

#define MY_LED_ORANGE   0x402200
#define MY_LED_REDMAG   0x400022
#define MY_LED_BLUECYAN 0x002240


//------------------------
// theClock definition
//------------------------

#define ID_RUNNING			"RUNNING"
#define ID_PID_MODE			"PID_MODE"
#define ID_PLOT_VALUES		"PLOT_VALUES"

#define ID_HALL_THRESH		"HALL_THRESH"

#define ID_POWER_LOW      	"POWER_LOW"
#define ID_POWER_HIGH      	"POWER_HIGH"
#define ID_POWER_MAX      	"POWER_MAX"
#define ID_POWER_START      "POWER_START"

#define ID_DUR_PULSE      	"DUR_PULSE"
#define ID_DUR_START		"DUR_START"

#define ID_PID_P			"PID_P"
#define ID_PID_I			"PID_I"
#define ID_PID_D			"PID_D"

#define ID_CLEAR_STATS		"CLEAR_STATS"

#define ID_CUR_TIME			"CUR_TIME"
#define ID_TIME_LAST_START  "LAST_START"
#define ID_STAT_RUNTIME		"TIME_RUNNING"
#define ID_STAT_BEATS		"BEATS"
#define ID_STAT_RESTARTS	"RESTARTS"
#define ID_STAT_STALLS_L	"STALLS_LEFT"
#define ID_STAT_STALLS_R	"STALLS_RIGHT"
#define ID_STAT_ERROR_L		"ERROR_LOW"
#define ID_STAT_ERROR_H		"ERROR_HIGH"
#define ID_STAT_DUR_L		"DUR_LOW"
#define ID_STAT_DUR_H		"DUR_HIGH"
#define ID_MIN_POWER_USED	"MIN_POWER_USED"
#define ID_MAX_POWER_USED	"MAX_POWER_USED"


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
	static uint32_t _plot_values;

	static int _hall_thresh;

	static int _power_low;		// STATIC: power when sensor reached; PID: mininum power
	static int _power_high;		// STATIC: power when sensor not reached;  PID: starting power
	static int _power_max;      // PID: maximum power
	static int _power_start;    // power during startup pulse

	static int _dur_pulse;		// duration of power pulse for both PID and STATIC
	static int _dur_start;		// duration of startup pulse

	static float _pid_P;
	static float _pid_I;
	static float _pid_D;

    static uint32_t _time_last_start;
	static uint32_t _cur_time;
	static String _stat_time_running;
	static uint32_t _stat_beats;
	static uint32_t _stat_restarts;
	static uint32_t _stat_stalls_left;
	static uint32_t _stat_stalls_right;
	static int _stat_error_low;
	static int _stat_error_high;
	static int _stat_dur_low;
	static int _stat_dur_high;
	static uint32_t _min_power_used;
	static uint32_t _max_power_used;

	static void startClock();
	static void stopClock();
	static void clearStats();

    static void onClockRunningChanged(const myIOTValue *desc, bool val);
    static void onPIDModeChanged(const myIOTValue *desc, bool val);

	static void run();
	static void clockTask(void *param);
};


extern theClock *the_clock;
