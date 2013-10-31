/*

*/

// Define include statements
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include "Adafruit_MAX31855.h"
// We are modifying this code
// Define library look ups
Adafruit_MAX31855 thermocouple (thermoCLK, thermoCS,  thermoDO);
Adafruit_MAX31855 thermocouple1(thermoCLK, thermoCS1, thermoDO);

// Define pins and pin modes
int thermoDO = 3;
int thermoCLK = 4;
int thermoCS = 5;
int thermoCS1 = 6;

// Define prototypes
void ErrorCheckingRoutine();

// Define set-up loop
void setup()
{
   Serial.begin( 9600);
} 

// Main loop
void main()
{
   // Error checking
   ErrorCheckingRoutine();
}

// Prototypes
void ErrorCheckingRoutine()
{

}










