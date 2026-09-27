#include <Arduino.h>
#include <STEPPER.h>
#include <STM32FreeRTOS.h>

// Définition des broches
// #define PIN_STBY D7
// #define PIN_STEP D6
// #define PIN_DIR D3
// #define PIN_EN A2
// #define PIN_M0 PB5
// #define PIN_M1 D9
// #define PIN_M2 D8
// #define PIN_FDC_HAUT PA1
// #define PIN_FDC_BAS PA4

#define PIN_STBY PA5
#define PIN_STEP PA4
#define PIN_DIR PA3
#define PIN_EN PB1
#define PIN_M0 PB0
#define PIN_M1 PA7
#define PIN_M2 PA6

#define PIN_FDC_HAUT PA1 // J8
#define PIN_FDC_BAS PA8 // J3
// FDC HAUT 

// Variables de contrôle moteur
bool STBY, STEP, DIR, EN, M0, M1, M2;

void initStepper()
{
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  digitalWrite(PIN_DIR, LOW);

  digitalWrite(PIN_STBY, HIGH); // sortir de veille
  delay(5);

  digitalWrite(PIN_EN, HIGH); // enable moteur
}

void haut(int pas)
{
  digitalWrite(PIN_DIR, LOW);
  for (int i = 0; i < pas; i++)
  {
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(500);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(500);
  }
}



void bas(int pas)
{
  digitalWrite(PIN_DIR, HIGH);
  for (int i = 0; i < pas; i++)
  {
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(400);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(400);
  }
}