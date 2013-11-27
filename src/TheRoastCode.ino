/*
  Arduino Coffee Roaster Controller
  Written by Andrew Schwartzmeyer
  Roaster designed and built by Erik Illum

  https://github.com/illumfour/coffee-roaster

  Heavily inspired and aided by Lim Phang Moh's "Reflow Oven
  Controller" - https://github.com/rocketscream/Reflow-Oven-Controller

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
   - Get button input for: increase time, change roast
   - Add logging components via Serial/shield
   - Run code by Dr. Rinker, Erik, Jordan
   - Add display: NHD-0420D3Z-FL-GBW
   - Add LED status lights
     - 5v LED use 100 Ohm
     - 12v LED use 470 Ohm

   Wiring:
     +/- to power supply (red/black)
     two heating relays (blue)
     one motor relay (blue)
     one fan relay (blue)
     heat gun:
       green alligator from fan relay ground (black)
       red to PSU
       thick white to heat gun power
       wall black to heat relay small
       heat relay big heat gun thick black

     LEDs resistor between high of relay and high of LED, then ground

 */

#define SERIAL

#ifdef SERIAL

#define DEBUG
#define TEMPS

#endif

#include "Adafruit_MAX31855.h"

/* enums for states */
typedef enum ROAST_STATE {
  ROAST_IDLE,
  ROAST_PREHEAT,
  ROAST_RAMP,
  ROAST_FULL,
  ROAST_COOLING,
  ROAST_COUNT,
} roastState_t;

typedef enum HEAT_STATE {
  HEAT_IDLE,
  HEAT_FULL,
} heatState_t;

typedef enum FAN_STATE {
  FAN_IDLE,
  FAN_PARTIAL,
  FAN_FULL,
} fanState_t;

typedef enum MOTOR_STATE {
  MOTOR_IDLE,
  MOTOR_FULL,
} motorState_t;

/* state variables */
roastState_t roast_state = ROAST_IDLE;
heatState_t heat_state = HEAT_IDLE;
fanState_t fan_state = FAN_IDLE;
motorState_t motor_state = MOTOR_IDLE;

/* pins */
const int BUTTON_PIN = 8;  /* Digital */
const int FAN_PIN = 9;  /* PWM one of Digital 3, 5, 6, 9, 10, or 11 */
const int MOTOR_PIN = 10;  /* PWM */
const int HEAT_PIN0 = 11;  /* Digital */
const int HEAT_PIN1 = 12;  /* Digital */

const int THERMO_DO = 3;  /* Digital */
const int THERMO_CS0 = 4;  /* Digital */
const int THERMO_CLK = 5;  /* Digital */
const int THERMO_CS1 = 6;  /* Digital */

/* constants */
const int RAMP_STEPS = 40;  /* divisor of ramp time and temp */

const int TEMP_COOL = 75;
const int TEMP_READY = 100;
const int TEMP_MAX = 250;
const int TEMP_STEP = 5;

const int ROAST_TIME = 20;  /* minutes */
const double HEAT_FULL_TIME = 0.25 * ROAST_TIME;
const double RAMP_TIME_STEP = (double)HEAT_FULL_TIME / RAMP_STEPS;
const double RAMP_HEAT_STEP = (double)(TEMP_MAX - TEMP_READY) / RAMP_STEPS;
const int SENSOR_SAMPLING_TIME = 1000;  /* ms */
const double FAN_FULL_TIME = 0.5 * ROAST_TIME;

const int FAN_PARTIAL_PERCENT = 50;
const int FAN_FULL_PERCENT = 100;

/* thermocouples */
const int NUM_THERMO = 1;  /* how many thermocouples */
Adafruit_MAX31855 thermocouple0(THERMO_CLK, THERMO_CS0, THERMO_DO);
Adafruit_MAX31855 thermocouple1(THERMO_CLK, THERMO_CS1, THERMO_DO);
Adafruit_MAX31855 temps[] = {thermocouple0, thermocouple1};

/* variables */
unsigned long start_time = 0;
unsigned long next_read = 0;
unsigned long target_time = 0;
unsigned long elapsed_time = 0;
unsigned long roast_start = 0;
unsigned long last_change = 0;
double internal_temp = 0;
double target_temp = TEMP_READY;

/* prototypes */
unsigned long min_to_ms(double minutes);
double ms_to_min(unsigned long ms);
int percent_to_duty(int percent);
void set_heat(uint8_t state);
void set_motor(uint8_t state);
void set_fan(int percent);
double get_temp(int i);
double get_avg_temp();

/* math functions */
unsigned long min_to_ms(double minutes) {
  return minutes * (unsigned long)60000;
}

double ms_to_min(unsigned long ms) {
  return (double)ms / 60000;
}

int percent_to_duty(int percent) {
  /* transforms percent to approximate PWM duty cycle */
  return 255*(percent/100.);
}

/* set targets */
void set_target_time() {
  target_time = elapsed_time + min_to_ms(RAMP_TIME_STEP);
}

void set_target_heat() {
  target_temp += RAMP_HEAT_STEP;
}

/* hardware control */

/* heat control */
void set_heat(uint8_t state) {
  /* accepts HIGH/LOW for ON/OFF */
  digitalWrite(HEAT_PIN0, state);
  digitalWrite(HEAT_PIN1, state);
}

void heat_idle() {
  if (heat_state != HEAT_IDLE) {
    set_heat(LOW);
    heat_state = HEAT_IDLE;
#ifdef DEBUG
    Serial.print(millis());
    Serial.print(": Idling heat\n");
#endif
  }
}

void heat_full() {
  if (heat_state != HEAT_FULL) {
    set_heat(HIGH);
    heat_state = HEAT_FULL;
#ifdef DEBUG
    Serial.print(millis());
    Serial.print(": Turning on heat\n");
#endif
    if (fan_state == FAN_IDLE) {
      /* ensures fan is at least at partial */
      fan_partial();
#ifdef DEBUG
      Serial.print(millis());
      Serial.print(": Turning fan to partial\n");
#endif
    }
  }
}
/* fan control */
void set_fan(int percent) {
  /* PWM f ~= 490 Hz */
  analogWrite(FAN_PIN, percent_to_duty(percent));
}

void fan_idle() {
  if (fan_state != FAN_IDLE) {
    set_fan(0);
    fan_state = FAN_IDLE;
#ifdef DEBUG
    Serial.print(millis());
    Serial.print(": Idling fan\n");
#endif
  }
}

void fan_partial() {
  if (fan_state != FAN_PARTIAL) {
    set_fan(FAN_PARTIAL_PERCENT);
    fan_state = FAN_PARTIAL;
#ifdef DEBUG
    Serial.print(millis());
    Serial.print(": Setting fan to partial\n");
#endif
 }
}

void fan_full() {
  if (fan_state != FAN_FULL) {
    set_fan(FAN_FULL_PERCENT);
    fan_state = FAN_FULL;
#ifdef DEBUG
    Serial.print(millis());
    Serial.print(": Setting fan to full\n");
#endif
  }
}

/* motor control */
void set_motor(uint8_t state) {
  /* accepts HIGH/LOW for ON/OFF */
  digitalWrite(MOTOR_PIN, state);
}

void motor_idle() {
  if (motor_state != MOTOR_IDLE) {
    set_motor(LOW);
    motor_state = MOTOR_IDLE;
#ifdef DEBUG
    Serial.print(millis());
    Serial.print(": Idling motor\n");
#endif
  }
}

void motor_full() {
  if (motor_state != MOTOR_FULL) {
    set_motor(HIGH);
    motor_state = MOTOR_FULL;
#ifdef DEBUG
    Serial.print(millis());
    Serial.print(": Turning motor on\n");
#endif
  }
}

/* thermocouple access */
double get_temp(int i) {
  double temp = temps[i].readCelsius();
  if (isnan(temp)) {
#ifdef TEMPS
    Serial.print(millis());
    Serial.print(": Thermocouple error ");
    Serial.print(i);
    Serial.println();
#endif
    return 0;
  } else {
#ifdef TEMPS
    Serial.print(millis());
    Serial.print(": Temp ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.print(temp);
    Serial.println();
#endif
    return temp;
  }
}

double get_avg_temp() {
  double average = 0;
  for (int i = 0; i < NUM_THERMO; i++) {
    average += get_temp(i);
  }
#ifdef TEMPS
  Serial.print(millis());
  Serial.print(": Average = ");
  Serial.print(average/NUM_THERMO);
  Serial.println();
#endif
  return average/NUM_THERMO;
}

/* roast control */
void roast_idle() {
  if (roast_state != ROAST_IDLE) {
    roast_state = ROAST_IDLE;
    heat_idle();
    fan_idle();
    motor_idle();
#ifdef DEBUG
    Serial.print(millis());
    Serial.print(": Idling roaster\n");
#endif
  }
}

void advance_roast() {
  /* handle on-advance code */
  switch (roast_state) {
  case ROAST_IDLE:
#ifdef SERIAL
    Serial.print(ms_to_min(elapsed_time));
    Serial.print(": Starting roast, entering pre-heat state\n");
#endif
    break;
  case ROAST_PREHEAT:
    roast_start = elapsed_time;  /* begin the roast */
    set_target_time();  /* set initial target */
#ifdef SERIAL
    Serial.print(ms_to_min(elapsed_time));
    Serial.print(": Pre-heat done, entering ramp state\n");
#endif
    break;
  case ROAST_RAMP:
#ifdef SERIAL
    Serial.print(ms_to_min(elapsed_time));
    Serial.print(": Ramping done, entering full-heat state\n");
#endif
    break;
  case ROAST_FULL:
#ifdef SERAL
    Serial.print(ms_to_min(elapsed_time));
    Serial.print(": Roast done, entering cooldown state\n");
#endif
    break;
  case ROAST_COOLING:
    start_time = elapsed_time;  /* reset start_time for next roast */
#ifdef SERIAL
    Serial.print(ms_to_min(elapsed_time));
    Serial.print(": Cooldown done, entering idle state\n");
#endif
    break;
  }
  /* cycle forward the roast state */
  roast_state = (roastState_t)((roast_state + 1) % ROAST_COUNT);
}

/* set-up Arduino */
void setup() {
#ifdef SERIAL
  Serial.begin(9600);
  Serial.print(millis());
  Serial.print(": System on\n");
#endif

/* set appropriate pin modes */
  pinMode(BUTTON_PIN, INPUT);
  pinMode(HEAT_PIN0, OUTPUT);
  pinMode(HEAT_PIN1, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);

/* begin idling */
  roast_idle();
  delay(500);  /* let things settle */

  start_time = millis();
  next_read = start_time;
#ifdef DEBUG
  Serial.print(start_time);
  Serial.print(" = start time\n");
#endif
} 

/* main Arduino loop */
void loop() {
  elapsed_time = millis() - start_time;

  /* advance roast state on button push */
  if (digitalRead(BUTTON_PIN) == HIGH) {
    if (elapsed_time > last_change) {
      advance_roast();
      last_change = elapsed_time + 500;
#ifdef DEBUG
      Serial.print(millis());
      Serial.print(": Button pushed\n");
#endif
    }
  }

  /* read sensor */
  if (elapsed_time > next_read) {
    next_read += SENSOR_SAMPLING_TIME;
    internal_temp = get_avg_temp();
#ifdef SERIAL
    Serial.println(ms_to_min(elapsed_time));
#endif
  }

  switch (roast_state) {
  case ROAST_IDLE:
    heat_idle();
    fan_idle();
    motor_idle();
    break;
  case ROAST_PREHEAT:
    fan_partial();
    motor_full();
    if (internal_temp < (TEMP_READY - TEMP_STEP)) {
      /* turn on heat if not hot enough */
      heat_full();
    } else {
      /* TODO Indicate ready */
    }
    if (internal_temp > TEMP_READY) {
      /* turn off heat and motor if hot enough */
      heat_idle();
    }
    break;
  case ROAST_RAMP:
    motor_full();
    if (elapsed_time > min_to_ms(HEAT_FULL_TIME)) {
      /* advance roast at appropriate time */
      advance_roast();
    } else if (elapsed_time > target_time) {
      /* increase targets RAMP_STEPS times, turn on heat if needed */
      set_target_time();
      set_target_heat();
      if (internal_temp < target_temp) heat_full();
#ifdef DEBUG
      Serial.print(millis());
      Serial.print(": Adding ramp/interval step\n");
      Serial.print("New target_time: ");
      Serial.print(target_time);
      Serial.println();
      Serial.print("New target_temp: ");
      Serial.print(target_temp);
      Serial.println();
#endif
    } else if (internal_temp > target_temp) heat_idle();
    break;
  case ROAST_FULL:
    /* maintain temperature at TEMP_MAX */
    if (internal_temp > TEMP_MAX) heat_idle();
    else if (internal_temp < (TEMP_MAX - TEMP_STEP)) heat_full();

    /* max fan at appropriate time */
    if (elapsed_time > min_to_ms(FAN_FULL_TIME)) fan_full();

    /* if ROAST_TIME reached, start cooling */    
    if ((elapsed_time - roast_start) > min_to_ms(ROAST_TIME)) advance_roast();
    break;
  case ROAST_COOLING:
    /* turn off heat, max fans until cool */
    heat_idle();
    fan_full();
    motor_full();
    if (internal_temp < TEMP_COOL) {
    /* TODO Indicate cooldown is done */
      advance_roast();
    }
    break;
  }
}
