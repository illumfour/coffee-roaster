/*
  Arduino Coffee Roaster Controller
  Written by Andrew Schwartzmeyer
  Roaster designed and built by Erik Illum

  https://github.com/illumfour/coffee-roaster

  Heavily inspired and aided by Lim Phang Moh's "Reflow Oven
  Controller" - https://github.com/rocketscream/Reflow-Oven-Controller

  Required pins:

  1 pin: boolean, rules heat gun + both elements
  1 pin: boolean, motor, but on throughout
  1 pin: PWM, fan

  Maximum relay switches = 100 times per roast

  Temperature
        |
  100% -|       ********%%%%%%%%%%%%%%%%%
        |      *        #
   75% -|     *         #
        |    *          #
   50% -|###%############
        |  *
   25% -| *
	|*
   0%  ----------------------------------
        |       |       |       |       |
   Time 0      25      50      75      100

   TODO:
   - Get button input for: start/stop, increase time, change roast
   - Add logging components via Serial/shield
   - Run code by Dr. Rinker, Erik, Jordan
   - Add LED status lights

 */

#define DEBUG
#define TEMPS

#include "Adafruit_MAX31855.h"

// enums

typedef enum ROAST_STATE {
  ROAST_IDLE,
  ROAST_PREHEAT,
  ROAST_ROASTING,
  ROAST_COOLING,
} roastState_t;

typedef enum HEAT_STATE {
  HEAT_IDLE,
  HEAT_PRE,
  HEAT_RAMP,
  HEAT_FULL,
} heatState_t;

typedef enum FAN_STATE {
  FAN_IDLE,
  FAN_PARTIAL,
  FAN_FULL,
} fanState_t;

typedef enum MOTOR_STATE {
  MOTOR_ON,
  MOTOR_OFF,
} motorState_t;

// states
roastState_t roast_state = ROAST_IDLE;
heatState_t heat_state = HEAT_IDLE;
fanState_t fan_state = FAN_IDLE;
motorState_t motor_state = MOTOR_OFF;

// pins
const int FAN_PIN = 9;  // PWM one of 3, 5, 6, 9, 10, or 11
const int HEAT_PIN = 10;  // Whatever we choose
const int MOTOR_PIN = 11;  // Ditto

const int THERMO_DO = 3;
const int THERMO_CLK = 4;
const int THERMO_CS0 = 5;
const int THERMO_CS1 = 6;

// constants
const int TEMP_COOL = 50;  // Especially arbitrary
const int TEMP_READY = 100;
const int TEMP_MAX = 250;
const int TEMP_STEP = 5;

const int ROAST_TIME = 20;  // minutes
const int RAMP_STEPS = 40;  // divisor of ramp time
const int HEAT_FULL_TIME = 0.25 * ROAST_TIME;
const int RAMP_TIME_INTERVAL = HEAT_FULL_TIME / RAMP_STEPS;
const int RAMP_HEAT_INTERVAL = (TEMP_MAX - TEMP_READY) / RAMP_STEPS;
const int SENSOR_SAMPLING_TIME = 1000;  // ms
const int FAN_FULL_TIME = 0.5 * ROAST_TIME;
const int FAN_PARTIAL_PERCENT = 50;
const int FAN_FULL_PERCENT = 100;

const int MS_IN_MINUTE = 60000;

// thermocouples
const int INTERNAL_TEMP = 0;  // Which one is internal
Adafruit_MAX31855 thermocouple0 (THERMO_CLK, THERMO_CS0, THERMO_DO);
Adafruit_MAX31855 thermocouple1 (THERMO_CLK, THERMO_CS1, THERMO_DO);
Adafruit_MAX31855 temps[] = {thermocouple0, thermocouple1};

// variables
unsigned long start_time = 0;
unsigned long next_read = 0;
unsigned long target_time = 0;
double target_temp = TEMP_READY;

// prototypes
unsigned long minutes_to_ms(int minutes);
int percent_to_duty(int percent);
void set_heat(uint8_t state);
void set_motor(uint8_t state);
void set_fan(int percent);
double get_temp(int i);

// helper functions
unsigned long minutes_to_ms(int minutes) {
  unsigned long total_ms = minutes * MS_IN_MINUTE;
  return total_ms;
}

double ms_to_minutes (unsigned long ms) {
  double total_ms = ms / MS_IN_MINUTE;
  return total_ms;
}

int percent_to_duty(int percent) {
  // transforms percent to approximate PWM duty cycle
  return 255*(percent/100);
}

// hardware control
void set_heat(uint8_t state) {
  // accepts HIGH/LOW for ON/OFF
  digitalWrite(HEAT_PIN, state);
}

void heat_idle() {
  if (heat_state != HEAT_IDLE) {
#ifdef DEBUG
    Serial.print(ms_to_minutes(millis()));
    Serial.print(": Turning off heat\n");
#endif
    heat_off();
    heat_state = HEAT_IDLE;
  }
}

void heat_off() {
  set_heat(LOW);
}

void heat_on() {
  if (heat_state != HEAT_IDLE) {
#ifdef DEBUG
    Serial.print(ms_to_minutes(millis()));
    Serial.print(": Turning on heat\n");
#endif
    set_heat(HIGH);
  }
}

void set_fan(int percent) {
  // PWM f ~= 490 Hz
  analogWrite(FAN_PIN, percent_to_duty(percent));
}

void fan_idle() {
  if (fan_state != FAN_IDLE) {
#ifdef DEBUG
    Serial.print(ms_to_minutes(millis()));
    Serial.print(": Setting fan to zero\n");
#endif
    set_fan(0);
    fan_state = FAN_IDLE;
  }
}

void fan_partial() {
  if (fan_state != FAN_PARTIAL) {
#ifdef DEBUG
    Serial.print(ms_to_minutes(millis()));
    Serial.print(": Setting fan to partial\n");
#endif
    set_fan(FAN_PARTIAL_PERCENT);
    fan_state = FAN_PARTIAL;
 }
}

void fan_full() {
  if (fan_state != FAN_FULL) {
#ifdef DEBUG
    Serial.print(ms_to_minutes(millis()));
    Serial.print(": Settin fan to full\n");
#endif
    set_fan(FAN_FULL_PERCENT);
    fan_state = FAN_FULL;
  }
}

void set_motor(uint8_t state) {
  // accepts HIGH/LOW for ON/OFF
  digitalWrite(MOTOR_PIN, state);
}

void motor_off() {
  if (motor_state != MOTOR_OFF) {
#ifdef DEBUG
    Serial.print(ms_to_minutes(millis()));
    Serial.print(": Turning motor off\n");
#endif
    set_motor(LOW);
    motor_state = MOTOR_OFF;
  }
}

void motor_on() {
  if (motor_state != MOTOR_ON) {
#ifdef DEBUG
    Serial.print(ms_to_minutes(millis()));
    Serial.print(": Turning motor on\n");
#endif
    set_motor(HIGH);
    motor_state == MOTOR_ON;
  }
}

double get_temp(int i) {
  double temp = temps[i].readCelsius();
  if (isnan(temp)) {
#ifdef TEMPS
    Serial.print(ms_to_minutes(millis()));
    Serial.println(": Thermocouple error ");
    Serial.println(i);
    Serial.println("\n");
    Serial.println("Shutting down\n");
#endif
    roast_state = ROAST_IDLE;
    return 0;
  } else {
#ifdef TEMPS
    Serial.print(ms_to_minutes(millis()));
    Serial.println(": Temp ");
    Serial.println(i);
    Serial.println(" = ");
    Serial.println(temp);
    Serial.println("\n");
#endif
    return temp;
  }
}

// Define set-up loop
void setup() {
  Serial.begin(9600);
  Serial.print(ms_to_minutes(millis()));
  Serial.print(": System on\n");

  pinMode(HEAT_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);

  delay(500);

  start_time = millis();
  next_read = start_time;
#ifdef DEBUG
  Serial.print(start_time);
  Serial.print(" = start time\n");
#endif

} 

// Main loop
void loop() {
  unsigned long elapsed_time = millis() - start_time;
  double internal_temp = 0;

  if (elapsed_time > next_read) {
#ifdef DEBUG
    Serial.print(ms_to_minutes(millis()));
    Serial.print(": Reading sensors\n");
#endif
    next_read += SENSOR_SAMPLING_TIME;
    internal_temp = get_temp(INTERNAL_TEMP);
  }

  switch (roast_state) {
  case ROAST_IDLE:
#ifdef DEBUG
    Serial.print(ms_to_minutes(millis()));
    Serial.print(": ROAST_IDLE\n");
#endif
    heat_idle();
    fan_idle();
    motor_off();
    // Indicate cooldown is done
    break;
  case ROAST_PREHEAT:
#ifdef DEBUG
    Serial.print(ms_to_minutes(millis()));
    Serial.print(": ROAST_PREHEAT\n");
#endif
    motor_on();
    heat_state = HEAT_PRE;
  case ROAST_ROASTING:
#ifdef DEBUG
    Serial.print(ms_to_minutes(millis()));
    Serial.print(": ROAST_ROASTING\n");
#endif
    motor_on();
    if (fan_state == FAN_IDLE) fan_partial();
    if (elapsed_time > minutes_to_ms(ROAST_TIME)) {
      roast_state = ROAST_COOLING;
    }
    break;
  case ROAST_COOLING:
    // Turn off heat, max fans until cool
#ifdef DEBUG
    Serial.print(ms_to_minutes(millis()));
    Serial.print(": ROAST_COOLING\n");
#endif
    heat_idle();
    fan_full();
    if (internal_temp < TEMP_COOL) roast_state = ROAST_IDLE;
    break;
  }

  switch (heat_state) {
  case HEAT_IDLE:
#ifdef DEBUG
    Serial.print(ms_to_minutes(millis()));
    Serial.print(": HEAT_IDLE\n");
#endif
    break;
  case HEAT_PRE:
    // Get temperature to TEMP_READY
#ifdef DEBUG
    Serial.print(ms_to_minutes(millis()));
    Serial.print(": HEAT_PRE\n");
#endif
    heat_on();
    if (internal_temp > TEMP_READY) {
      start_time = elapsed_time;  // Restart timer
      target_time = elapsed_time;
      roast_state = ROAST_ROASTING;
      heat_state = HEAT_RAMP;
      // Indicate pre-heating is done
    }
    break;
  case HEAT_RAMP:
    // Gradually increase temperature to TEMP_MAX
#ifdef DEBUG
    Serial.print(ms_to_minutes(millis()));
    Serial.print(": HEAT_RAMP\n");
#endif
    if (elapsed_time > minutes_to_ms(HEAT_FULL_TIME)) {
      heat_state = HEAT_FULL;
    } else if (elapsed_time > target_time) {
      // Increase targets RAMP_STEPS times, turn on heat if needed
      target_time += minutes_to_ms(RAMP_TIME_INTERVAL);
      target_temp += RAMP_HEAT_INTERVAL;
      if (internal_temp < target_temp) heat_on();
    } else if (internal_temp > target_temp) heat_off();
    break;
  case HEAT_FULL:
    // Maintain temperature at TEMP_MAX
#ifdef DEBUG
    Serial.print(ms_to_minutes(millis()));
    Serial.print(": HEAT_FULL\n");
#endif
    if (internal_temp > TEMP_MAX) heat_off();
    else if (internal_temp < (TEMP_MAX - TEMP_STEP)) heat_on;
    break;
  }

  switch (fan_state) {
  case FAN_IDLE:
#ifdef DEBUG
    Serial.print(ms_to_minutes(millis()));
    Serial.print(": FAN_IDLE\n");
#endif
    break;
  case FAN_PARTIAL:
#ifdef DEBUG
    Serial.print(ms_to_minutes(millis()));
    Serial.print(": FAN_PARTIAL\n");
#endif
    if (elapsed_time > minutes_to_ms(FAN_FULL_TIME)) fan_full();
    break;
  case FAN_FULL:
#ifdef DEBUG
    Serial.print(ms_to_minutes(millis()));
    Serial.print(": FAN_FULL\n");
#endif
    break;
  }

  switch (motor_state) {
  case MOTOR_OFF:
#ifdef DEBUG
    Serial.print(ms_to_minutes(millis()));
    Serial.print(": MOTOR_OFF\n");
#endif
    break;
  case MOTOR_ON:
#ifdef DEBUG
    Serial.print(ms_to_minutes(millis()));
    Serial.print(": MOTOR_ON\n");
#endif
    break;
  }
}
