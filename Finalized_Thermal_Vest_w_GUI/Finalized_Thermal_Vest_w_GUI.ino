#include <OneWire.h>
#include <DallasTemperature.h>
#include "ArduPID.h"

#define ONE_WIRE_BUS 13
#define PELTIER_PIN1 5
#define PELTIER_PIN2 4
#define PELTIER_PIN3 3
#define PELTIER_PIN4 2
#define THERM_PIN A0
#define THERMISTORPIN_1 A1
#define THERMISTORPIN_2 A2
#define THERMISTORPIN_3 A3
#define THERMISTORPIN_4 A4
#define THERMISTORPIN_5 A5

//Ear Thermistor Constants
#define Therm_NR 2559 // resistance at 25 degrees C in Ohm
#define TempN 23 // temp. for nominal resistance (almost always 25 C)
#define NUMSAMPLES 10 // how many samples to take and average, more takes longer
#define B 3950 // The beta coefficient of the thermistor (usually 3000-4000)
#define Fixed_R 10000 // the value of the 'other' resistor in Ohm

//Body Thermistor Constants
#define THERMISTORNOMINAL 10000 // resistance at 25 degrees C
#define TEMPERATURENOMINAL 25 // temp. for nominal resistance (almost always 25 C)
#define NUMSAMPLES 10 //number of samples
#define BCOEFFICIENT 3950 // The beta coefficient of the thermistor (usually 3000-4000)
#define SERIESRESISTOR 10000 // the value of the 'other' resistor

//Establish sample arrays
int samples[NUMSAMPLES];
int samples_1[NUMSAMPLES];
int samples_2[NUMSAMPLES];
int samples_3[NUMSAMPLES];
int samples_4[NUMSAMPLES];
int samples_5[NUMSAMPLES];

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
ArduPID myController;

bool cellState1 = true;
bool cellState2 = true;
bool cellState3 = true;
bool cellState4 = true;

void turnOnCell1();
void turnOffCell1();

void turnOnCell2();
void turnOffCell2();

void turnOnCell3();
void turnOffCell3();

void turnOnCell4();
void turnOffCell4();


//Controller Variables
double input;
double output;
double prevOutput = 1;
double setPoint = 15; // temperature in C
double prevSetPoint = setPoint;
double p = 100;
double i = 100;
double d = 10;

float therm_gain = 1 / .01;

unsigned long beginTime;

void setup() {
  pinMode(PELTIER_PIN1, OUTPUT); //Set Peltier Pins as Outputs
  pinMode(PELTIER_PIN2, OUTPUT);
  pinMode(PELTIER_PIN3, OUTPUT);
  pinMode(PELTIER_PIN4, OUTPUT);

  pinMode(THERM_PIN, INPUT); //Set Thermistor Pins as Inputs
  pinMode(THERMISTORPIN_1, INPUT);
  pinMode(THERMISTORPIN_2, INPUT);
  pinMode(THERMISTORPIN_3, INPUT);
  pinMode(THERMISTORPIN_4, INPUT);

  Serial.begin(115200);
  analogReference(EXTERNAL);
  sensors.begin(); //Start up the library
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
  float average_1 = 0;
  float average_2 = 0;
  float average_3 = 0;
  float average_4 = 0;
  float average_5 = 0;

  for (i = 0; i < NUMSAMPLES; i++) { // take N samples in a row, with a slight delay
    samples[i] = analogRead(THERM_PIN);
    analogRead(THERMISTORPIN_1);
    delay(10);
    samples_1[i] = analogRead(THERMISTORPIN_1);
    analogRead(THERMISTORPIN_2); //should clear S and H buffer
    delay(10);
    samples_2[i] = analogRead(THERMISTORPIN_2);
    analogRead(THERMISTORPIN_3);
    delay(10);
    samples_3[i] = analogRead(THERMISTORPIN_3);
    analogRead(THERMISTORPIN_4);
    delay(10);
    samples_4[i] = analogRead(THERMISTORPIN_4);
    analogRead(THERMISTORPIN_5);
    delay(10);
    samples_5[i] = analogRead(THERMISTORPIN_5);
    delay(10);
  }

  for (i = 0; i < NUMSAMPLES; i++) {
    average_1 += samples_1[i];
    average_2 += samples_2[i];
    average_3 += samples_3[i];
    average_4 += samples_4[i];
    average_5 += samples_5[i];
  }

  average_1 /= NUMSAMPLES; //Average samples
  average_2 /= NUMSAMPLES;
  average_3 /= NUMSAMPLES;
  average_4 /= NUMSAMPLES;
  average_5 /= NUMSAMPLES;

  average_1 = 1023 / average_1 - 1; //convert the value to resistance
  average_2 = 1023 / average_2 - 1;
  average_3 = 1023 / average_3 - 1;
  average_4 = 1023 / average_4 - 1;
  average_5 = 1023 / average_5 - 1;

  average_1 = SERIESRESISTOR / average_1;
  average_2 = SERIESRESISTOR / average_2;
  average_3 = SERIESRESISTOR / average_3;
  average_4 = SERIESRESISTOR / average_4;
  average_5 = SERIESRESISTOR / average_5;

  float steinhart_1;
  float steinhart_2;
  float steinhart_3;
  float steinhart_4;
  float steinhart_5;

  steinhart_1 = average_1 / THERMISTORNOMINAL; //(R/Ro)
  steinhart_2 = average_2 / THERMISTORNOMINAL;
  steinhart_3 = average_3 / THERMISTORNOMINAL;
  steinhart_4 = average_4 / THERMISTORNOMINAL;
  steinhart_5 = average_5 / THERMISTORNOMINAL;

  steinhart_1 = log(steinhart_1); // ln(R/Ro)
  steinhart_2 = log(steinhart_2);
  steinhart_3 = log(steinhart_3);
  steinhart_4 = log(steinhart_4);
  steinhart_5 = log(steinhart_5);

  steinhart_1 /= BCOEFFICIENT; // 1/B * ln(R/Ro)
  steinhart_2 /= BCOEFFICIENT;
  steinhart_3 /= BCOEFFICIENT;
  steinhart_4 /= BCOEFFICIENT;
  steinhart_5 /= BCOEFFICIENT;

  steinhart_1 += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart_2 += 1.0 / (TEMPERATURENOMINAL + 273.15);
  steinhart_3 += 1.0 / (TEMPERATURENOMINAL + 273.15);
  steinhart_4 += 1.0 / (TEMPERATURENOMINAL + 273.15);
  steinhart_5 += 1.0 / (TEMPERATURENOMINAL + 273.15);

  steinhart_1 = 1.0 / steinhart_1; // Invert
  steinhart_2 = 1.0 / steinhart_2;
  steinhart_3 = 1.0 / steinhart_3;
  steinhart_4 = 1.0 / steinhart_4;
  steinhart_5 = 1.0 / steinhart_5;

  steinhart_1 -= 273.15; // convert absolute temp to C
  steinhart_2 -= 273.15;
  steinhart_3 -= 273.15;
  steinhart_4 -= 273.15;
  steinhart_5 -= 273.15;

  float core_temp = readEarTherm(samples);

  sensors.requestTemperatures();
  double water_temp = sensors.getTempCByIndex(0);
  input = water_temp;

  
  Serial.print((millis() - beginTime) / 1000);
  Serial.print(",");
  Serial.print(sensors.getTempCByIndex(0)); // Why "byIndex"?
  Serial.print(",");
  Serial.print(core_temp);
  Serial.print(",");
  Serial.print(steinhart_1);
  Serial.print(",");
  Serial.print(steinhart_2);
  Serial.print(",");
  Serial.print(steinhart_3);
  Serial.print(",");
  Serial.print(steinhart_4);
  Serial.print(",");
  Serial.print(steinhart_5);
  Serial.print(",");
  Serial.print(cellState1);
  Serial.print(",");
  Serial.print(cellState2);
  Serial.print(",");
  Serial.print(cellState3);
  Serial.print(",");
  Serial.print(cellState4);
  Serial.print(",");
  Serial.println(setPoint);
  

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

  if (output > 50 && !(prevOutput <= 50) || core_temp < 30 || steinhart_1 < 20 || steinhart_2 < 20 || steinhart_3 < 20 || steinhart_4 < 20 || steinhart_5 < 20) {
    turnOffCell1();
    turnOffCell2();
    turnOffCell3();
    turnOffCell4();

  } else if (output > 30 && !(prevOutput <= 30)) {
    turnOnCell1();
    turnOnCell2();
    turnOffCell3();
    turnOffCell4();

  } else if (output > 10 && !(prevOutput <= 10)) {
    turnOnCell1();
    turnOnCell2();
    turnOnCell3();
    turnOffCell4();

  } else {
    turnOnCell1();
    turnOnCell2();
    turnOnCell3();
    turnOnCell4();
  }

  prevOutput = output;
  delay(50);
}

//Functions
void turnOnCell1() {
  digitalWrite(PELTIER_PIN1, HIGH);
  cellState1 = true;
}
void turnOffCell1() {
  digitalWrite(PELTIER_PIN1, LOW);
  cellState1 = false;
}

void turnOnCell2() {
  digitalWrite(PELTIER_PIN2, HIGH);
  cellState2 = true;
}
void turnOffCell2() {
  digitalWrite(PELTIER_PIN2, LOW);
  cellState2 = false;
}

void turnOnCell3() {
  digitalWrite(PELTIER_PIN3, HIGH);
  cellState3 = true;
}
void turnOffCell3() {
  digitalWrite(PELTIER_PIN3, LOW);
  cellState3 = false;
}

void turnOnCell4() {
  digitalWrite(PELTIER_PIN4, HIGH);
  cellState4 = true;
}
void turnOffCell4() {
  digitalWrite(PELTIER_PIN4, LOW);
  cellState4 = false;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float readEarTherm(int reading[]) {
  float average;
  uint8_t i;
  average = 0; // average all the samples out
  for (i = 0; i < NUMSAMPLES; i++) {
    average += reading[i];
  }
  average /= NUMSAMPLES;
  average = 1023 / average - 1; // convert the value to resistance
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
