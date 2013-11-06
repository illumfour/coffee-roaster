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
        0      25      50      75      100
 */

#include "Adafruit_MAX31855.h"

// enums

typedef enum ROAST_STATE {
  ROAST_IDLE,
  ROAST_ROASTING,
  ROAST_ERROR,
} roastState_t;

typedef enum HEAT_STATE {
  HEAT_IDLE,
  HEAT_RAMP,
  HEAT_FULL,
} heatState_t;

typedef enum FAN_STATE {
  FAN_IDLE,
  FAN_HALF,
  FAN_FULL,
} fanState_t;

// states
roastState_t roast_state = ROAST_IDLE;
heatState_t heat_state = HEAT_IDLE;
fanState_t fan_state = FAN_IDLE;

// pins
const int FAN_PIN = 9;  // PWM one of 3, 5, 6, 9, 10, or 11
const int HEAT_PIN = 10;  // Whatever we choose
const int MOTOR_PIN = 11;  // Ditto

const int THERMO_DO = 3;
const int THERMO_CLK = 4;
const int THERMO_CS0 = 5;
const int THERMO_CS1 = 6;

// constants
const int ROAST_TIME = 2;
const int TEMP_MAX = 250;
const int TEMP_STEP = 5;
const int RAMP_STEP_MICRO = 30;  // Min = 12
const int SENSOR_SAMPLING_TIME = 1;
const int MS_IN_MINUTE = 60000;

// thermocouples
Adafruit_MAX31855 thermocouple0 (THERMO_CLK, THERMO_CS0, THERMO_DO);
Adafruit_MAX31855 thermocouple1 (THERMO_CLK, THERMO_CS1, THERMO_DO);
Adafruit_MAX31855 temps[] = {thermocouple0, thermocouple1};


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

int percent_to_duty(int percent) {
  // transforms percent to approximate PWM duty cycle
  return 255*(percent/100);
}

// hardware control
void set_heat(uint8_t state) {
  // accepts HIGH/LOW for ON/OFF
  digitalWrite(HEAT_PIN, state);
}

void set_motor(uint8_t state) {
  // accepts HIGH/LOW for ON/OFF
  digitalWrite(MOTOR_PIN, state);
}

void set_fan(int percent) {
  // PWM f ~= 490 Hz
  analogWrite(FAN_PIN, percent_to_duty(percent));
}

double get_temp(int i) {
  double temp = temps[i].readCelsius();
  if (isnan(temp)) {
    Serial.println("Thermocouple error ");
    Serial.println(i);
    Serial.println("\n");
    Serial.println("Shutting down\n");
    roast_state = ROAST_ERROR;
    return 0;
  } else {
    Serial.println("Temp ");
    Serial.println(i);
    Serial.println(": ");
    Serial.println(temp);
    Serial.println("at ");
    Serial.println(millis()/1000);
    Serial.println("seconds.\n");
    return temp;
  }
}

double get_avg_temp() {
  double temp = 0;
  for (int i = 0; i < 2; i++) temp += get_temp(i);
  return temp/2;
}

// Define set-up loop
void setup() {
  Serial.begin(9600);

  pinMode(HEAT_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);

  set_heat(LOW);
  set_fan(LOW);
  set_motor(LOW);
  
  delay(500);
} 

// Main loop
void loop() {
  unsigned long start_time;
}
