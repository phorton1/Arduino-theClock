// the_clock.ino
//
// contains the myIOT device definition, setup() and loop()

#include "theClock.h"
#include <myIOTLog.h>

//------------------------
// theClock definition
//------------------------

#define THE_CLOCK             "theClock"
#define THE_CLOCK_VERSION     "1.0"



#define DEFAULT_POWER_LOW		180
#define DEFAULT_POWER_HIGH		200
#define DEFAULT_POWER_SPRING	128
#define DEFAULT_POWER_PULL		255

#define DEFAULT_DUR_LEFT		60
#define DEFAULT_DUR_RIGHT		100
#define DEFAULT_DUR_START		60
#define DEFAULT_DUR_PULL		200

#define DEFAULT_PID_P			1.40
#define DEFAULT_PID_I			0.20
#define DEFAULT_PID_D			0.00


// what shows up on the "dashboard" UI tab

static valueIdType dash_items[] = {
	ID_RUNNING,
	ID_PID_MODE,
    ID_REBOOT,

	ID_CLEAR_STATS,
	ID_TIME_LAST_START,
	ID_STAT_TIME,
	ID_STAT_BEATS,
	ID_STAT_RESTARTS,
	ID_STAT_STALLS_L,
	ID_STAT_STALLS_R,
	ID_STAT_OVER_L,
	ID_STAT_OVER_R,
	ID_STAT_ERROR_L,
	ID_STAT_ERROR_H,

	0,
};


// what shows up on the "device" UI tab

static valueIdType device_items[] = {
	ID_POWER_LOW,
    ID_POWER_HIGH,
    ID_POWER_SPRING,
	ID_POWER_PULL,
    ID_DUR_LEFT,
	ID_DUR_RIGHT,
	ID_DUR_START,
	ID_DUR_PULL,
	ID_PID_P,
	ID_PID_I,
	ID_PID_D,
    0
};


// value descriptors for testDevice



const valDescriptor theClock::m_clock_values[] =
{
    { ID_DEVICE_NAME,      VALUE_TYPE_STRING,    VALUE_STORE_PREF,     VALUE_STYLE_REQUIRED,   NULL,   NULL,   THE_CLOCK },
        // DEVICE_NAME overrides base class element

	{ ID_RUNNING,      		VALUE_TYPE_BOOL,     VALUE_STORE_PREF,     VALUE_STYLE_NONE,       (void *) &_clock_running,(void *) onClockRunningChanged, },
	{ ID_PID_MODE,      	VALUE_TYPE_BOOL,     VALUE_STORE_PREF,     VALUE_STYLE_NONE,       (void *) &_pid_mode, 	(void *) onPIDModeChanged, },

	{ ID_POWER_LOW,  		VALUE_TYPE_INT,      VALUE_STORE_PREF,     VALUE_STYLE_NONE,       (void *) &_power_low,	NULL,  { .int_range = { DEFAULT_POWER_LOW,  	0,  255}} },
	{ ID_POWER_HIGH,  		VALUE_TYPE_INT,      VALUE_STORE_PREF,     VALUE_STYLE_NONE,       (void *) &_power_high,	NULL,  { .int_range = { DEFAULT_POWER_HIGH,   	0,  255}} },
	{ ID_POWER_SPRING,  	VALUE_TYPE_INT,      VALUE_STORE_PREF,     VALUE_STYLE_NONE,       (void *) &_power_spring, NULL,  { .int_range = { DEFAULT_POWER_SPRING,   0,  255}} },
	{ ID_POWER_PULL,  		VALUE_TYPE_INT,      VALUE_STORE_PREF,     VALUE_STYLE_NONE,       (void *) &_power_pull,   NULL,  { .int_range = { DEFAULT_POWER_PULL,     0,  255}} },
	{ ID_DUR_LEFT,  		VALUE_TYPE_INT,      VALUE_STORE_PREF,     VALUE_STYLE_NONE,       (void *) &_dur_left,		NULL,  { .int_range = { DEFAULT_DUR_LEFT,   	0,  255}} },
	{ ID_DUR_RIGHT,  		VALUE_TYPE_INT,      VALUE_STORE_PREF,     VALUE_STYLE_NONE,       (void *) &_dur_right,	NULL,  { .int_range = { DEFAULT_DUR_RIGHT,   	0,  255}} },
	{ ID_DUR_START,  		VALUE_TYPE_INT,      VALUE_STORE_PREF,     VALUE_STYLE_NONE,       (void *) &_dur_start,	NULL,  { .int_range = { DEFAULT_DUR_START,   	0,  255}} },
	{ ID_DUR_PULL,  		VALUE_TYPE_INT,      VALUE_STORE_PREF,     VALUE_STYLE_NONE,       (void *) &_dur_pull,		NULL,  { .int_range = { DEFAULT_DUR_PULL,   	0,  255}} },

	{ ID_PID_P,  			VALUE_TYPE_FLOAT,    VALUE_STORE_PREF,     VALUE_STYLE_NONE,       (void *) &_pid_P,		NULL,  { .float_range = { DEFAULT_PID_P,   		0,  10}} },
	{ ID_PID_I,  			VALUE_TYPE_FLOAT,    VALUE_STORE_PREF,     VALUE_STYLE_NONE,       (void *) &_pid_I,		NULL,  { .float_range = { DEFAULT_PID_I,   		0,  10}} },
	{ ID_PID_D,  			VALUE_TYPE_FLOAT,    VALUE_STORE_PREF,     VALUE_STYLE_NONE,       (void *) &_pid_D,		NULL,  { .float_range = { DEFAULT_PID_D,   	  -10,  10}} },

	{ ID_CLEAR_STATS,       VALUE_TYPE_COMMAND,  VALUE_STORE_MQTT_SUB, VALUE_STYLE_NONE,       NULL,                    (void *) clearStats },
	{ ID_TIME_LAST_START,   VALUE_TYPE_TIME,     VALUE_STORE_PUB,      VALUE_STYLE_READONLY,   (void *) &_time_last_start, },
	{ ID_STAT_TIME,      	VALUE_TYPE_STRING,   VALUE_STORE_PUB,      VALUE_STYLE_READONLY,   (void *) &_stat_time, },

	{ ID_STAT_BEATS,  		VALUE_TYPE_INT,      VALUE_STORE_PUB,      VALUE_STYLE_READONLY,   (void *) &_stat_beats, 		NULL,  { .int_range = { 0, -DEVICE_MAX_INT-1,DEVICE_MAX_INT}} },
	{ ID_STAT_RESTARTS,  	VALUE_TYPE_INT,      VALUE_STORE_PUB,      VALUE_STYLE_READONLY,   (void *) &_stat_restarts, 	NULL,  { .int_range = { 0, -DEVICE_MAX_INT-1,DEVICE_MAX_INT}} },
	{ ID_STAT_STALLS_L,  	VALUE_TYPE_INT,      VALUE_STORE_PUB,      VALUE_STYLE_READONLY,   (void *) &_stat_stalls_left, NULL,  { .int_range = { 0, -DEVICE_MAX_INT-1,DEVICE_MAX_INT}} },
	{ ID_STAT_STALLS_R,  	VALUE_TYPE_INT,      VALUE_STORE_PUB,      VALUE_STYLE_READONLY,   (void *) &_stat_stalls_right,NULL,  { .int_range = { 0, -DEVICE_MAX_INT-1,DEVICE_MAX_INT}} },
	{ ID_STAT_OVER_L,  		VALUE_TYPE_INT,      VALUE_STORE_PUB,      VALUE_STYLE_READONLY,   (void *) &_stat_over_left,   NULL,  { .int_range = { 0, -DEVICE_MAX_INT-1,DEVICE_MAX_INT}} },
	{ ID_STAT_OVER_R,  		VALUE_TYPE_INT,      VALUE_STORE_PUB,      VALUE_STYLE_READONLY,   (void *) &_stat_over_right, 	NULL,  { .int_range = { 0, -DEVICE_MAX_INT-1,DEVICE_MAX_INT}} },
	{ ID_STAT_ERROR_L,  	VALUE_TYPE_INT,      VALUE_STORE_PUB,      VALUE_STYLE_READONLY,   (void *) &_stat_error_low, 	NULL,  { .int_range = { 0, -DEVICE_MAX_INT-1,DEVICE_MAX_INT}} },
	{ ID_STAT_ERROR_H,  	VALUE_TYPE_INT,      VALUE_STORE_PUB,      VALUE_STYLE_READONLY,   (void *) &_stat_error_high, 	NULL,  { .int_range = { 0, -DEVICE_MAX_INT-1,DEVICE_MAX_INT}} },

};



#define NUM_CLOCK_VALUES (sizeof(m_clock_values)/sizeof(valDescriptor))

// static member data

bool 	theClock::_clock_running = 1;
bool 	theClock::_pid_mode = 1;

int  	theClock::_power_low;
int  	theClock::_power_high;
int  	theClock::_power_spring;
int  	theClock::_power_pull;

int  	theClock::_dur_left;
int  	theClock::_dur_right;
int  	theClock::_dur_start;
int  	theClock::_dur_pull;

float  	theClock::_pid_P;
float  	theClock::_pid_I;
float  	theClock::_pid_D;

uint32_t theClock::_time_last_start;
String   theClock::_stat_time;
uint32_t theClock::_stat_beats;
uint32_t theClock::_stat_restarts;
uint32_t theClock::_stat_stalls_left;
uint32_t theClock::_stat_stalls_right;
uint32_t theClock::_stat_over_left;
uint32_t theClock::_stat_over_right;
int      theClock::_stat_error_low;
int      theClock::_stat_error_high;


// ctor

theClock::theClock()
{
    addValues(m_clock_values,NUM_CLOCK_VALUES);
    setTabLayouts(dash_items,device_items);
}


//--------------------------------
// main
//--------------------------------

theClock *the_clock;


void setup()
{
    Serial.begin(115200);
    delay(1000);

    theClock::setDeviceType(THE_CLOCK);
    theClock::setDeviceVersion(THE_CLOCK_VERSION);

    LOGU("");
    LOGU("");
    LOGU("the_clock.ino setup() started on core(%d)",xPortGetCoreID());

    the_clock = new theClock();
    the_clock->setup();

    LOGU("the_clock.ino setup() finished");
}



void loop()
{
    the_clock->loop();
}
