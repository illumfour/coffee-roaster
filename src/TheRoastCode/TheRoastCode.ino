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
#define CSV

#endif

#ifdef LOGGER

#include "SD.h"
#include <Wire.h>
#include "RTClib.h"

#endif

#include <math.h>
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
  HEAT_COUNT,
} heatState_t;

typedef enum MOTOR_STATE {
  MOTOR_IDLE,
  MOTOR_FULL,
  MOTOR_COUNT,
} motorState_t;

/* state variables */
roastState_t roast_state = ROAST_COUNT;
heatState_t heat_state = HEAT_COUNT;
motorState_t motor_state = MOTOR_COUNT;
bool ready = false;

/* pins */
const int HEAT_PIN0 = 10;  /* Digital */
const int HEAT_PIN1 = 11;  /* Digital */
const int MOTOR_PIN = 12;  /* PWM */

const int THERMO_CLK = 4;  /* Digital */
const int THERMO_DO = 5;  /* Digital */
const int THERMO_CS0 = 6;  /* Digital */
const int THERMO_CS1 = 7;  /* Digital */

const int TIME_BUTTON = 8;  /* Digital */
const int STATE_BUTTON = 9;  /* Digital */
#ifdef LOGGER
const int DATA_LOG_PIN = 10;  /* Digital */
#endif
/* note that SD communication takes pins 11, 12, and 13 as per
   http://arduino.cc/en/Reference/SDCardNotes */

/* constants */
const int SAMPLE_SIZE = 4;
const int ERROR_MARGIN = 3;
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
const char COMMA = ',';

/* thermocouples */
const int NUM_THERMO = 1;  /* how many thermocouples */
Adafruit_MAX31855 thermocouple0(THERMO_CLK, THERMO_CS0, THERMO_DO);
Adafruit_MAX31855 thermocouple1(THERMO_CLK, THERMO_CS1, THERMO_DO);
Adafruit_MAX31855 temps[] = {thermocouple0, thermocouple1};

/* data logger */
#ifdef LOGGER
File log_file;
char log_file_name[] = "LOGGER00.CSV";
RTC_DS1307 RTC;
bool log_flag = true;
#endif

// prototypes
unsigned long min_to_ms(double minutes);

/* variables */
unsigned long roast_time = min_to_ms(ROAST_TIME);
unsigned long start_time = 0;
unsigned long next_read = 0;
unsigned long target_time = 0;
unsigned long elapsed_time = 0;
unsigned long roast_start = 0;
unsigned long last_change = 0;
double internal_temp = 0.0;
double target_temp = TEMP_READY;

/* sample collection */
double samples[SAMPLE_SIZE];
int sample_index = 0;

/* prototypes */
unsigned long min_to_ms(double minutes);
double ms_to_min(unsigned long ms);
int percent_to_duty(int percent);
void set_heat(uint8_t state);
void set_motor(uint8_t state);
double get_temp(int i);
double get_avg_temp();
double calculate_mean(double data[], int size);
double calculate_standard_deviation(double data[], double mean, int size);
void add_sample();

/* math functions */
unsigned long min_to_ms(double minutes) {
  return minutes * (unsigned long)60000;
}

double ms_to_min(unsigned long ms) {
  return (double)ms / 60000;
}

int percent_to_duty(int percent) {
  /* transforms percent to approximate PWM duty cycle */
  return 255 * (percent / 100.);
}

double calculate_mean(double data[], int size) {
  /* calculates the mean of an array of data */
  double mean = 0.0;
  for (int i = 0; i < size; i++) mean += data[i];
  return mean / size;
}

double calculate_standard_deviation(double data[], double mean, int size) {
  /* calculates the standard deviation of an array of data */
  double stddev_sum = 0.0;
  double stddev = 0.0;
  for (int i = 0; i < size; i++) stddev_sum += square(data[i] - mean);
  stddev = sqrt(stddev_sum / size);
  if (stddev < 3) stddev = 3;
  return stddev;
}

/* sample collection */
void add_sample() {
  double mean = calculate_mean(samples, SAMPLE_SIZE);
  double stddev = calculate_standard_deviation(samples, mean, SAMPLE_SIZE);
  double sample = 0.0;
#ifdef DEBUG
  Serial.print(millis());
  Serial.print(": mean = ");
  Serial.print(mean);
  Serial.print(", stddev = ");
  Serial.println(stddev);
#endif
  /* replace sample with prior one if outside our margin of error */
  do {
    sample = get_avg_temp();
#ifdef DEBUG
    Serial.print(millis());
    Serial.print(": raw temp = ");
    Serial.println(sample);
#endif
  } while ((sample < 10.) || (sample > 500.) ||
	   (sample > (mean + (ERROR_MARGIN * stddev))) ||
	   (sample < (mean - (ERROR_MARGIN * stddev))));
  samples[sample_index] = sample;
  sample_index = (sample_index + 1) % SAMPLE_SIZE;
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
    Serial.println(": Idling heat");
#endif
  }
}

void heat_full() {
  if (heat_state != HEAT_FULL) {
    set_heat(HIGH);
    heat_state = HEAT_FULL;
#ifdef DEBUG
    Serial.print(millis());
    Serial.println(": Turning on heat");
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
    Serial.println(": Idling motor");
#endif
  }
}

void motor_full() {
  if (motor_state != MOTOR_FULL) {
    set_motor(HIGH);
    motor_state = MOTOR_FULL;
#ifdef DEBUG
    Serial.print(millis());
    Serial.println(": Turning motor on");
#endif
  }
}

/* thermocouple access */
double get_temp(int i) {
  double temp = 0;
  do {
    temp = temps[i].readCelsius();
    delay(10);
  } while (isnan(temp));
  return temp;
}

double get_avg_temp() {
  double average = 0.0;
  for (int i = 0; i < NUM_THERMO; i++) average += get_temp(i);
  return average/NUM_THERMO;
}

/* file handlers */
#ifdef logger
void open_log_file() {
  log_file = SD.open(log_file_name, FILE_WRITE);
}
#endif

/* roast control */
void roast_idle() {
  if (roast_state != ROAST_IDLE) {
    roast_state = ROAST_IDLE;
    heat_idle();
    motor_idle();
#ifdef DEBUG
    Serial.print(millis());
    Serial.println(": Idling roaster");
#endif
  }
}

void advance_roast() {
  /* handle on-advance code */
  switch (roast_state) {
  case ROAST_IDLE:
#ifdef SERIAL
    Serial.print(ms_to_min(elapsed_time));
    Serial.println(": Starting roast, entering pre-heat state");
#endif
    break;
  case ROAST_PREHEAT:
    roast_start = elapsed_time;  /* begin the roast */
    set_target_time();  /* set initial target */
#ifdef SERIAL
    Serial.print(ms_to_min(elapsed_time));
    Serial.println(": Pre-heat done, entering ramp state");
#endif
    break;
  case ROAST_RAMP:
#ifdef SERIAL
    Serial.print(ms_to_min(elapsed_time));
    Serial.println(": Ramping done, entering full-heat state");
#endif
    break;
  case ROAST_FULL:
#ifdef SERAL
    Serial.print(ms_to_min(elapsed_time));
    Serial.println(": Roast done, entering cooldown state");
#endif
    break;
  case ROAST_COOLING:
    start_time = elapsed_time;  /* reset start_time for next roast */
#ifdef SERIAL
    Serial.print(ms_to_min(elapsed_time));
    Serial.println(": Cooldown done, entering idle state");
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
  Serial.println(": System on");
#endif

  /* set appropriate pin modes */
#ifdef LOGGER
  pinMode(DATA_LOG_PIN, OUTPUT);
#endif
  pinMode(STATE_BUTTON, INPUT);
  pinMode(TIME_BUTTON, INPUT);
  pinMode(HEAT_PIN0, OUTPUT);
  pinMode(HEAT_PIN1, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);

  /* initialize SD card for data logger */
#ifdef LOGGER
  if (!SD.begin(DATA_LOG_PIN)) {
#ifdef SERIAL
    Serial.println("Card failed, or not present");
#endif
    log_flag = false;
  }
  if (log_flag) {
#ifdef SERIAL
    Serial.println("Card initialized");
#endif
    /* create a new file */
    for (uint8_t i = 0; i < 100; i++) {
      log_file_name[6] = i/10 + '0';
      log_file_name[7] = i%10 + '0';
      if (!SD.exists(log_file_name)) {
	/* only open a new file if it doesn't exist */
	open_log_file();
	Serial.println(log_file);
	break;  // leave the loop!
      }
    }
    if (!log_file) {
#ifdef SERIAL
      Serial.println("Failed write to log_file");
#endif
      log_flag = false;
    } else {
#ifdef SERIAL
      Serial.print("Logging to: ");
      Serial.println(log_file_name);
#endif
    }
  }
    /* setup RTC */
  if (log_flag) {
    Wire.begin();
    if (!RTC.begin()) log_file.println("RTC failed");
    log_file.close();
  }
#endif

  /* begin idling */
  roast_idle();
  delay(500);  /* let things settle */

  /* fill samples with data */
  for (int i = 0; i < SAMPLE_SIZE; i++) samples[i] = get_avg_temp();

  start_time = millis();
  next_read = start_time;
#ifdef DEBUG
  Serial.print(start_time);
  Serial.println(" = start time");
#endif
}

/* main Arduino loop */
void loop() {
#ifdef logger
  DateTime now;
  int percent = 0;
#endif
  elapsed_time = millis() - start_time;

  /* advance roast state on button push */
  if (digitalRead(STATE_BUTTON) == HIGH) {
    if (elapsed_time > last_change) {
      last_change = elapsed_time + 500;
      advance_roast();
#ifdef DEBUG
      Serial.print(millis());
      Serial.println(": State button pushed");
#endif
    }
  }

  /* time increase button */
  if (digitalRead(TIME_BUTTON) == HIGH) {
    if (elapsed_time > last_change) {
      last_change = elapsed_time + 500;
      roast_time += 10 * 1000;
#ifdef DEBUG
      Serial.print(millis());
      Serial.println(": Time button pushed");
      Serial.print("Roast time is ");
      Serial.print(ms_to_min(roast_time));
      Serial.println();
#endif
    }
  }

  /* read sensor */
  if (elapsed_time > next_read) {
    next_read += SENSOR_SAMPLING_TIME;
    add_sample();
    internal_temp = calculate_mean(samples, SAMPLE_SIZE);
#ifdef CSV
  Serial.print("@ ");
  Serial.print(ms_to_min(elapsed_time));
  Serial.print(COMMA);
  Serial.print(ms_to_min(elapsed_time - roast_start));
  Serial.print(COMMA);
  Serial.print(internal_temp);
  Serial.print(COMMA);
  Serial.println(roast_state);
#endif
#ifdef LOGGER
    if (log_flag) open_log_file();
    if (log_file) {
      now = RTC.now();
      log_file.print(now.unixtime());
      log_file.print(COMMA);
      log_file.print(now.year(), DEC);
      log_file.print("/");
      log_file.print(now.month(), DEC);
      log_file.print("/");
      log_file.print(now.day(), DEC);
      log_file.print(" ");
      log_file.print(now.hour(), DEC);
      log_file.print(":");
      log_file.print(now.minute(), DEC);
      log_file.print(":");
      log_file.print(now.second(), DEC);
      log_file.print(COMMA);
      log_file.print(ms_to_min(elapsed_time));
      log_file.print(COMMA);
      log_file.print(internal_temp);
      log_file.print(COMMA);
      log_file.print(get_temp(0));
      log_file.print(COMMA);
      log_file.print(get_temp(1));
      log_file.print(COMMA);
      log_file.println(roast_state);
      log_file.close();
    }
#endif
  }

  switch (roast_state) {
  case ROAST_IDLE:
    heat_idle();
    motor_idle();
    break;
  case ROAST_PREHEAT:
    motor_full();
    if (internal_temp < (TEMP_READY - TEMP_STEP)) {
      /* turn on heat if not hot enough */
      heat_full();
      if (ready) ready = false;
    }
    if (internal_temp > TEMP_READY) {
      /* turn off heat and motor if hot enough */
      heat_idle();
      if (!ready) {
	ready = true;
#ifdef SERIAL
      Serial.print(millis());
      Serial.println(": Ready");
#endif
      }
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
      Serial.println(": Adding ramp/interval step");
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

    /* if ROAST_TIME reached, start cooling */    
    if ((elapsed_time - roast_start) > roast_time) advance_roast();
    break;
  case ROAST_COOLING:
    /* turn off heat */
    heat_idle();
    motor_full();
    if (internal_temp < TEMP_COOL) {
    /* TODO Indicate cooldown is done */
      advance_roast();
    }
    break;
  }
}
