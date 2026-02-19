#include <Arduino.h>
#include "INA236.h"
#include "STM32_CAN.h"
#include "ID_CAN.h"

#define PIN_ALERT PA4
#define ADRESSE_INA 0x41

#define TENSION_CELLULE_DECHARGE 1.1 // Seuil d'une cellule est déchargé
#define TENSION_CELLULE_CHARGE 1.4   // tension pour laquelle une cellule est bien chargé

STM32_CAN Can1(CAN1, DEF); // Utilise les broches PA11/12 pour CAN1.3#
static CAN_message_t CAN_TX_msg;
static CAN_message_t CAN_RX_msg;


INA236 Batt(ADRESSE_INA);

char nbre_cellules;
float Vbatt,Ibatt;
float VbattCAN, IbattCAN;

unsigned long previousMillis = 0; // Variable pour stocker le dernier temps enregistré
const long interval = 5000;       // Intervalle de 5 secondes (5000 millisecondes)
ulong start_millis = 0;
ulong valid_millis = 0;


char calcul_nombre_cellules(float tension);
void envoi_int_CAN(int valeur, uint32_t id);

void setup()
{
  Serial.begin(9600);
  pinMode(PIN_ALERT, INPUT);


  // Initialisation CAN
  Can1.begin();
  Can1.setBaudRate(500000); // 1MBaud/s
  Wire.begin();

  // Init INA236
  Batt.begin();

  // Exemple : config shunt (adapte à ton montage)
  Batt.setMaxCurrentShunt(5.0, 0.005, true);  
  // courant max = 5 A, Rshunt = 5 mΩ
}

void loop()
{
  // Lecture INA236
  float Vbatt = Batt.getBusVoltage();     // en volts
  float Ibatt = Batt.getCurrent_mA();     // en mA

  // Lecture ALERT si tu l’utilises
  int alertState = digitalRead(PIN_ALERT);

  nbre_cellules = calcul_nombre_cellules(Vbatt);

  VbattCAN = Vbatt * 100; //conversion pour envoi CAN (2 décimales)
  IbattCAN = Ibatt * 100; //conversion pour envoi CAN (2 décimales)

  unsigned long currentMillis = millis(); // Récupère le temps actuel
  
  // Vérifie si 5 secondes se sont écoulées
  if (currentMillis - previousMillis >= interval)
  {
    Serial.printf("Envoi CAN des données de batterie...");
    envoi_int_CAN((int)VbattCAN, V_BATT);
    envoi_int_CAN((int)IbattCAN, I_BATT);
    
    previousMillis = currentMillis; // Met à jour le temps de référence

  }

  //envoi_int_CAN(VbattCAN, V_BATT);
  //envoi_int_CAN(IbattCAN, I_BATT);
    

  // Debug
  Serial.printf("V = ");
  Serial.print(Vbatt);
  Serial.printf(" V | I = ");
  Serial.print(Ibatt);
  Serial.printf(" mA | ALERT = ");
  Serial.println(alertState);

  delay(500);
}

char calcul_nombre_cellules(float tension)
{
  char nombre_elements;
  if ((tension > 10 * TENSION_CELLULE_DECHARGE) && (tension < 10 * TENSION_CELLULE_CHARGE))
  { // cas 10 cellules 12-14 V
    nombre_elements = 10;
  }
  else if ((tension > 6 * TENSION_CELLULE_DECHARGE) && (tension < 6 * TENSION_CELLULE_CHARGE))
  { // cas 6 cellules 6.6-8.4 V
    nombre_elements = 6;
  }
  else
    nombre_elements = 0;
  return nombre_elements;
}

void envoi_int_CAN(int valeur, uint32_t id) 
{
    CAN_TX_msg.id = id;  // ID CAN pour le message
    CAN_TX_msg.len = 2;  // 2 octets pour un int16_t (ou 4 pour un int32_t)
    // Pour un int16_t (2 octets) :
    CAN_TX_msg.buf[0] = (valeur >> 8) & 0xFF;  // Octet haut
    CAN_TX_msg.buf[1] = valeur & 0xFF;         // Octet bas
    // Pour un int32_t (4 octets) :
    // CAN_TX_msg.buf[2] = (valeur >> 16) & 0xFF;
    // CAN_TX_msg.buf[3] = (valeur >> 24) & 0xFF;
    Can1.write(CAN_TX_msg);
}
