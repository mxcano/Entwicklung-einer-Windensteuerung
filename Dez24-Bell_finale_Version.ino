// Hinzufügen der Bibliotheken:
#include <Arduino.h> // Arduino-Bibliothek
#include <Wire.h> // Wire-Bibliothek
#include <Adafruit_MCP4725.h> // MCP4725-Bibliothek
#include <Stepper.h> // Schrittmotor-Bibliothek


// Definition der Pins und Objekte:
Adafruit_MCP4725 ZYL_DAC; // MCP4725-Modul des Zylindermotors
Adafruit_MCP4725 WIN_DAC; // MCP4725-Modul des Windenmotors

#define ZYL_F_R_SWITCH 38 // Richtungsschalter des Zylindermotors
#define WIN_F_R_SWITCH 39 // Richtungsschalter des Windenmotors
#define POTI 2 // Potentiometer des Dreharm

#define HALL 1 // Hall-Sensor an Trommelwandung
#define PULSE 13 // Pulse-Pin für den Schrittmotor
#define DIRECTION 14 // Direction-Pin für den Schrittmotor
#define STEPPER_ZERO 42 // Schalter am Anfang der Linearführung


// Definition der verwendeten Konstanten und Variabeln:
const uint8_t zyl_I2C_address = 0x61; // I2C-Adresse des Zylindermotors
const uint8_t win_I2C_address = 0x60; // I2C-Adresse des Windenmotors

const float min_voltage = 1.2; // minimale Eingangsspannung der Drehzahlregler (in V)
const float max_voltage = 2.6; // maximale Eingangsspannung der Drehzahlregler (in V)
const float dac_min_voltage = min_voltage / (5 / 4096.0); // geringste Eingangsspannung der Drehzahlregler als DAC-Wert
const float dac_max_voltage = max_voltage / (5 / 4096.0); // höchste Eingangsspannung der Drehzahlregler als DAC-Wert

int16_t new_zyl_speed = 0; // neue Geschwindigkeit für den Zylindermotor (von -1000 bis +1000)
int16_t new_win_speed = 0; // neue Geschwindigkeit für den Windenmotor (von -1000 bis +1000)

unsigned long speed_set_millis = millis(); // Zeit seit letzter Geschwindigkeitsausgabe an die Motoren
const uint16_t speed_set_intervall = 10; // Intervall für die Geschwindigkeitsausgabe an die Motoren

const float i = 10.8/4.91; // Übersetzungsverhältnis Motoren
const float n = 2 // Wert der die Empfindlichkeit im Ausdruck d angibt
const uint16_t min_poti = 1850; // Potentiometerstellung ohne Zug
const uint16_t max_poti = 350; // Potentiometerstellung mit Zug
const uint16_t opt_poti = (min_poti + max_poti) / 2; // optimale Potentiometerstellung


const uint16_t steps_per_revolution = 200 * 4; // Schrittzahl für eine vollständige Umdrehung des Schrittmotors (4x Microstepping)
const uint16_t part_revolution = steps_per_revolution / 25; // Schrittzahl pro vorbeidrehendem Magneten (≙ 1 mm):
const uint16_t stepper_speed = 100; // Drehgeschwindigkeit des Schrittmotors (in U/min)
volatile unsigned long stepper_count = 0; // Schrittzähler
const float max_steps = (422 / 50.0) * steps_per_revolution; // Länge der Linearführung in Schritten (l = 422 mm)
volatile bool stepper_zero = false; // Schrittmotors genullt? → TRUE sobald Schrittmotor genullt wurde


// Objektinitialisierung Schrittmotor:
Stepper linStepper(steps_per_revolution, PULSE, DIRECTION);


// Initialisierung der verwendeten Funktionen:
void Set_Zyl_Speed(); // Set_Zyl_Speed()-Funktion
void Set_Win_Speed(); // Set_Win_Speed()-Funktion
void Adjust_Win_Speed(); // Adjust_Win_Speed()-Funktion

void Stepper_Interrupt(); // Interrupt-Funktion des Schrittmotors



// setup()-Funktion des Programms:
void setup() {

  // Initialisierung der benötigten Pins:
  pinMode(ZYL_F_R_SWITCH, OUTPUT);
  pinMode(WIN_F_R_SWITCH, OUTPUT);
  pinMode(POTI, INPUT);
  pinMode(HALL, INPUT);
  pinMode(STEPPER_ZERO, INPUT);
  

  // Initialisierung der seriellen und I2C-Kommunikation: 
  Serial.begin(115200);
  Wire.begin(4, 5);
  Wire.setClock(100000);
  analogReadResolution(12);


  // Initialisierung der MCP4725-Module:
  if (ZYL_DAC.begin(zyl_I2C_address)) {
    Serial.println("ZYL-DAC initialization successful!");
  } 
  else {
    Serial.println("ZYL-DAC initialization failed!");
  }

  if (WIN_DAC.begin(win_I2C_address)) {
    Serial.println("WIN-DAC initialization successful!");
  } 
  else {
    Serial.println("WIN-DAC initialization failed!");
  }


  // Festlegen des Schrittmotor-Interrupts:
  attachInterrupt(digitalPinToInterrupt(HALL), Stepper_Interrupt, RISING);

}



// loop()-Funktion des Programms:
void loop() {

  int16_t temp_speed = 0;  //temporärer Speicherplatz für Speedwerte aus dem seriellen Monitor für if-Abfrage (damit in Wertebereich)

  // Eingabe neuer Geschwndigkeitswerte über den seriellen Monitor:
  if (Serial.available()) {
    temp_speed = Serial.parseInt();

    if (temp_speed <= 1000 && temp_speed >= -1000) {
      new_zyl_speed = temp_speed;

      Serial.print("speed = ");
      Serial.println(new_zyl_speed);
    }
    else {
      Serial.println("speed value is not within the defined range (-1000 to 1000)");
    }

  }

  // periodische Ausgabe der aktuellen Geschwindigkeit an die Motoren
  if (millis() - speed_set_millis > speed_set_intervall) {
    
    Adjust_Win_Speed();
    Set_Zyl_Speed(new_zyl_speed);
    Set_Win_Speed(new_win_speed);

    speed_set_millis = millis();

  }

}



// Set_Zyl_Speed()-Funktion:
void Set_Zyl_Speed(int16_t new_speed) {

  // Definition der relevanten Variablen für die Funktion:
  static int16_t current_speed = 0;
  bool direction = 1;
  static uint16_t dac_value = 0;

  // Fallunterscheidung der Geschwindigkeitsänderung:
  if (new_speed > current_speed) {
    current_speed++;
  }
  if (new_speed < current_speed) {
    current_speed--;
  }

  // Abfrage ob Richtungsänderung vorgenommen werden muss:
  if (current_speed >= 0) {
    direction = 1;
  }
  else {
    direction = 0;
  }

  dac_value = map(abs(current_speed), 0, 1000, (int)dac_min_voltage, (int)dac_max_voltage);

  ZYL_DAC.setVoltage(dac_value, false);
  digitalWrite(ZYL_F_R_SWITCH, direction);

}



// Set_Win_Speed()-Funktion:
void Set_Win_Speed(int16_t new_speed) {

  // Definition der relevanten Variablen für die Funktion:
  static int16_t current_speed = 0;
  bool direction = 1;
  static uint16_t dac_value = 0;

  // Abfrage nach der Art der Geschwindigkeitsänderung:
  if (new_speed > current_speed) {
    current_speed++;
  }
  if (new_speed < current_speed) {
    current_speed--;
  }

  if (current_speed >= 0) {
    direction = 1;
  }
  else {
    direction = 0;
  }

  dac_value = map(abs(current_speed), 0, 1000, (int)dac_min_voltage, (int)dac_max_voltage);

  WIN_DAC.setVoltage(dac_value, false);
  digitalWrite(WIN_F_R_SWITCH, direction);

}



// Adjust_Win_Speed()-Funktion:
void Adjust_Win_Speed() {

  current_poti = analogRead(POTI);

  // Formel zur Berechnung der neuen Windenmotorgeschwindigkeit:
  new_win_speed = -1*(1/i * ((n-1)+(current_poti/opt_poti))/n * new_zyl_speed);

}



// Interrupt-Funktion des Schrittmotors:
void Stepper_Interrupt() {

  // Referenzfahrt des Schrittmotors (wenn stepper_zero = False):
  if (stepper_zero == false) {
    if (digitalRead(STEPPER_ZERO) == HIGH) {
      stepper_zero = true;
      part_revolution = -part_revolution;
      stepper_count = 0;
    }
  }

  // Richtungsänderung des Schrittmotors:
  if (stepper_count >= max_steps) {
    part_revolution = -part_revolution;
    stepper_count = 0;
  }

  // normaler Betrieb des Schrittmotors:
  linStepper.step(part_revolution);
  stepper_count += abs(part_revolution);

}