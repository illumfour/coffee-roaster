/*
  1 pin: boolean, rules heat gun + both elements
  1 pin: boolean, motor, but on throughout
  1 pin: PWM, fan

  minumum switching time = 20 / 100
 */

#include "Adafruit_MAX31855.h"

// pins
const int fan_pin = 9;  // PWM one of 3, 5, 6, 9, 10, or 11
const int heat_pin = 10;  // Whatever we choose
const int motor_pin = 11;  // Ditto

const int thermo_DO = 3;
const int thermo_CLK = 4;
const int thermo_CS0 = 5;
const int thermo_CS1 = 6;

// constants
const int ms_in_minute = 60000;

// thermocouples
Adafruit_MAX31855 temps = [thermocouple0(thermo_CLK, thermo_CS0, thermo_DO), thermocouple1(thermo_CLK, thermo_CS1, thermo_DO)];

// prototypes
unsigned long minutes_to_ms(int minutes);

// helper functions
unsigned long minutes_to_ms(int minutes) {
  unsigned long total_ms = minutes * ms_in_minute;
  return total_ms;
}

// hardware control
void set_heat(uint8_t state) {
  pinMode(heat_pin, OUTPUT);
  digitalWrite(heat_pin, state);
}

void set_motor(unit8_t state) {
  pinMode(motor_pin, OUTPUT);
  digitalWrite(motor_pin, state);
}

void set_fan(int duty) {
  // duty in [0, 255]
  // PWM f ~= 490 Hz
  analogWrite(fan_pin, duty);
}

double get_temp(int couple) {
  double temp = temps[couple].readCelsius();
  if (isnan(c)) {
    Serial.println("Something wrong with thermocouple!");
  }
}

// Define set-up loop
void setup() {
  Serial.begin(9600);

  set_motor(HIGH);
  
  delay(500);
} 

// Main loop
void loop() {
}
