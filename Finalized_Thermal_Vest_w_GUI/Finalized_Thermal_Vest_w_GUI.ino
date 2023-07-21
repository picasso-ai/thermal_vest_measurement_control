#include <OneWire.h>
#include <DallasTemperature.h>
#include "ArduPID.h"

#define ONE_WIRE_BUS 2
#define LARGE_PELTIER_PIN1 3
#define LARGE_PELTIER_PIN2 4.
#define SMALL_PELTIER_PIN1 5
#define SMALL_PELTIER_PIN2 6.
#define THERM_PIN A0

// resistance at 25 degrees C in Ohm
#define Therm_NR 2559
// temp. for nominal resistance (almost always 25 C)
#define TempN 23
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define B 3950
// the value of the 'other' resistor in Ohm
#define Fixed_R 10000

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
ArduPID myController;

void turnOnSmallCell();
void turnOffSmallCell();

void turnOnLargeCell();
void turnOffLargeCell();


double input;
double output;
double prevOutput = 1;
double setPoint = 15; // temperature in C
double prevSetPoint = setPoint;
double p = 10;
double i = 10;
double d = 100;

float therm_gain = 1 / .01;
int samples[NUMSAMPLES];

bool cellState1 = true;
bool cellState2 = true;

unsigned long beginTime;

void setup() {
  // Set RelayPin as an output pin
  pinMode(LARGE_PELTIER_PIN1, OUTPUT);
  pinMode(LARGE_PELTIER_PIN2, OUTPUT);
  pinMode(SMALL_PELTIER_PIN1, OUTPUT);
  pinMode(SMALL_PELTIER_PIN2, OUTPUT);

  pinMode(THERM_PIN, INPUT);

  Serial.begin(115200);
  analogReference(EXTERNAL);
  // Start up the library
  sensors.begin();
  sensors.setResolution(12);


  myController.begin(&input, &output, &setPoint, p, i, d);

  myController.setOutputLimits(0, 100);
  myController.setBias(50);
  myController.setWindUpLimits(-10, 10); // Groth bounds for the integral term to prevent integral wind-up

  myController.start();

  beginTime = millis();
}


void loop(void)
{
  uint8_t i;

  // take N samples in a row, with a slight delay
  for (i = 0; i < NUMSAMPLES; i++) {
    samples[i] = analogRead(THERM_PIN);
    delay(10);
  }

  float core_temp = readEarTherm(samples);

  sensors.requestTemperatures();
  double water_temp = sensors.getTempCByIndex(0);
  input = water_temp;

  Serial.print(sensors.getTempCByIndex(0)); // Why "byIndex"?
  Serial.print(","); // Why "byIndex"?
  Serial.print(sensors.getTempCByIndex(1)); // Why "byIndex"?
  Serial.print(","); // Why "byIndex"?
  Serial.print(sensors.getTempCByIndex(2)); // Why "byIndex"?
  Serial.print(","); // Why "byIndex"?
  Serial.print(sensors.getTempCByIndex(3)); // Why "byIndex"?
  Serial.print(","); // Why "byIndex"?
  Serial.print(sensors.getTempCByIndex(4)); // Why "byIndex"?
  Serial.print(","); // Why "byIndex"?
  Serial.print(sensors.getTempCByIndex(5)); // Why "byIndex"?
  Serial.print(","); // Why "byIndex"?
  Serial.print(sensors.getTempCByIndex(6)); // Why "byIndex"?
  Serial.print(","); // Why "byIndex"?
  Serial.print(sensors.getTempCByIndex(7)); // Why "byIndex"?
  Serial.print(","); // Why "byIndex"?
  Serial.print(setPoint); // Why "byIndex"?
  Serial.print(","); // Why "byIndex"?
  Serial.print(cellState1);
  Serial.print(","); // Why "byIndex"?
  Serial.print(cellState2);
  Serial.print(","); // Why "byIndex"?
  Serial.print(core_temp); // Why "byIndex"?
  Serial.print(","); // Why "byIndex"?
  Serial.println((millis()-beginTime) / 1000); // Why "byIndex"?

  myController.compute();


  if (Serial.available()) {
    String teststr = Serial.readString();
    int code = teststr.charAt(0) - '0';
    if (code == 0) {
      teststr = teststr.substring(1, teststr.length());
      const char* newTempstr = teststr.c_str();
      double newTemp = atof(newTempstr);
      if (0 <= newTemp && newTemp <= 40) {
        setPoint = newTemp;
      }
    } else if (code == 1) {
      teststr = teststr.substring(1, teststr.length());

      const char* newPstr = teststr.c_str();
      double newP = atof(newPstr);
      p = newP;
      myController.setCoefficients(p, i, d);
    } else if (code == 2) {
      teststr = teststr.substring(1, teststr.length());

      const char* newIstr = teststr.c_str();
      double newI = atof(newIstr);
      i = newI;
      myController.setCoefficients(p, i, d);
    } else if (code == 3) {
      teststr = teststr.substring(1, teststr.length());

      const char* newDstr = teststr.c_str();
      double newD = atof(newDstr);
      d = newD;
      myController.setCoefficients(p, i, d);
    }
  }


  if (output > 50 && !(prevOutput <= 50)) {
    turnOffSmallCell();
    turnOffLargeCell();

  } else if (output > 30 && !(prevOutput <= 30)) {
    turnOffLargeCell();
    turnOnSmallCell();
  } else if (output > 10 && !(prevOutput <= 10)) {
    turnOffSmallCell();
    turnOnLargeCell();
  } else {
    turnOnSmallCell();
    turnOnLargeCell();
  }

  prevOutput = output;
  delay(50);
}

void turnOnSmallCell() {
  digitalWrite(SMALL_PELTIER_PIN1, HIGH);
  digitalWrite(SMALL_PELTIER_PIN2, LOW);
  cellState2 = true;
}

void turnOffSmallCell() {
  digitalWrite(SMALL_PELTIER_PIN1, LOW);
  digitalWrite(SMALL_PELTIER_PIN2, LOW);
  cellState2 = false;
}

void turnOnLargeCell() {
  digitalWrite(LARGE_PELTIER_PIN1, HIGH);
  digitalWrite(LARGE_PELTIER_PIN2, LOW);
  cellState1 = true;
}
void turnOffLargeCell() {
  digitalWrite(LARGE_PELTIER_PIN1, LOW);
  digitalWrite(LARGE_PELTIER_PIN2, LOW);
  cellState1 = false;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
float readEarTherm(int reading[]) {
  float average;
  uint8_t i;
  // average all the samples out
  average = 0;
  for (i = 0; i < NUMSAMPLES; i++) {
    average += reading[i];
  }
  average /= NUMSAMPLES;

  // convert the value to resistance
  average = 1023 / average - 1;
  average = Fixed_R / average;

  float steinhart;
  steinhart = average / Therm_NR;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= B;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TempN + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert absolute temp to C

  return steinhart;
}
