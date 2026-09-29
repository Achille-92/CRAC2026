#include <Arduino.h>
#include "STM32_CAN.h"
#include "INA236.h"
#include "ID_CAN.h"

#define PINARU PF1   // D8
#define PINALERT PA4 // A3
// attention interrupteur 1 et 3 inversée sur la sérigraphie PIN par rapport au schéma sur kicad
// pins par rapport a la sérigraphie
#define PININTERRUPTEURBATT1 PA7     // D6
#define PININTERRUPTEURBATT2 PB0     // D3
#define PININTERRUPTEURBATT3 PB1     // A6
#define TENSION_CELLULE_DECHARGE 1.1 // Seuil d'une cellule est déchargé
#define TENSION_CELLULE_CHARGE 1.4   // tension pour laquelle une cellule est bien chargé
// Adresses Capteurs des batteries
#define ADRESSE_CAPTMAIN 0x40
#define ADRESSE_CAPT1 0x43
#define ADRESSE_CAPT2 0x42
#define ADRESSE_CAPT3 0x41
// Pas dans l'ordre a cause d'une erreur de sérigraphie sur la carte
#define ON 1
#define OFF 0
/*
#define ALERTE_DECHARGE_BATT1 0x201
#define ALERTE_DECHARGE_BATT2 0x202
#define ALERTE_DECHARGE_BATT3 0x203
*/
INA236 BatterieMain(ADRESSE_CAPTMAIN);
INA236 Batterie1(ADRESSE_CAPT1);
INA236 Batterie2(ADRESSE_CAPT2);
INA236 Batterie3(ADRESSE_CAPT3);

STM32_CAN Can1(CAN1, DEF); // Utilise les broches PA11/12 pour CAN1.3#
static CAN_message_t CAN_TX_msg;
static CAN_message_t CAN_RX_msg;

int i;
char val_aru, val_alert, etat_interrupteur1 = OFF, etat_interrupteur2 = OFF, etat_interrupteur3 = OFF;
int idTrames_CAN_Recu, mode_actuel = 1, etat_RPI, step;
float valtest;
float VbattMAIN, IbattMAIN, VbattMAIN_decharge; // Tension, Courant et Tension pour laquelle la batterie est déchargé
float Vbatt1, Ibatt1, Vbatt1_decharge;
float Vbatt2, Ibatt2, Vbatt2_decharge;
float Vbatt3, Ibatt3, Vbatt3_decharge;
float Vbatt1_charge, Vbatt2_charge, Vbatt3_charge, VbattMAIN_charge;
float Vbatt1CAN, Vbatt2CAN, Vbatt3CAN;
char nbre_cellules_Main, nbre_cellules_1, nbre_cellules_2, nbre_cellules_3;
unsigned char Tension[4], Courant[4];
float energieConsommee = 0.0;       // énergie consommée en Wh
char Batt1, Batt2, Batt3, BattMAIN; // pourcentage batterie

unsigned long previousMillis = 0; // Variable pour stocker le dernier temps enregistré
const long interval = 1000;       // Intervalle de 1 secondes (1000 millisecondes)
ulong start_millis = 0;
ulong valid_millis = 0;

// float power_limit_W;
// int limit;

void conversion_float_to_4char(float nombre, unsigned char *adresse_tableau);
float conversion_4char_to_float(unsigned char *adresse_tableau);
char calcul_nombre_cellules(float tension);
void arret_urgence(void);
void envoi_tension_courant(float tension, float courant, int id);                                 // envoi des courant et tension dans le bus CAN
void detectionCC(float tension, float courant, char voie_batterie);                               // detecte un court circuit sur voie batterie 1, 2, 3
void envoi_tension_min_max(float tension_actuelle, float tension_min, float tension_max, int id); // envoi tension actuelle, min et max
void envoi_float_CAN(float valeur, uint32_t id);                                                  // envoi une valeur float dans le bus CAN
void envoi_int_CAN(int valeur, uint32_t id);                                                      // envoi une valeur int dans le bus CAN
void envoi_floatvir_CAN(float valeur, uint32_t id);                                               // envoi une valeur float dans le bus CAN sans conversion en 4 chars
void envoi_char_CAN(char valeur, uint32_t id);                                                    // envoi une valeur char dans le bus CAN
char Calcul_bat(float actuel, float charge, float decharge);                                      // calcul pourcentage batterie

void setup()
{
  Serial.begin(9600);
  // Initialisation des I/O TOR
  pinMode(PINARU, INPUT_PULLUP);
  pinMode(PINALERT, INPUT);
  pinMode(PININTERRUPTEURBATT1, OUTPUT);
  pinMode(PININTERRUPTEURBATT2, OUTPUT);
  pinMode(PININTERRUPTEURBATT3, OUTPUT);

  // Interruption aru front descendant
  attachInterrupt(digitalPinToInterrupt(PINARU), arret_urgence, RISING);

  // Initialisation CAN
  Can1.begin();
  Can1.setBaudRate(500000); // 0.5MBaud/s
  Wire.begin();

  // Filtres pour recevoir des message spécifiques
  // bank number, id, masque
  Can1.setFilter(0, INTERRUPTEUR_BATT1, 0x1FFFFFFF);
  Can1.setFilter(1, INTERRUPTEUR_BATT2, 0x1FFFFFFF);
  Can1.setFilter(2, INTERRUPTEUR_BATT3, 0x1FFFFFFF);
  Can1.setFilter(3, MODE, 0x1FFFFFFF);
  Can1.setFilter(4, RPI, 0x1FFFFFFF);


  // Initialisation batterie main
  BatterieMain.begin();
  BatterieMain.setMaxCurrentShunt(3, 0.00483, true); // Courant max en A et RSHUNT en ohm, normalisation
  // Initialisation batterie 1
  Batterie1.begin();
  Batterie1.setMaxCurrentShunt(3, 0.00471, true); // Courant max en A et RSHUNT en ohm, normalisation
  // Initialisation batterie 2
  Batterie2.begin();
  Batterie2.setMaxCurrentShunt(3, 0.00514, true); // Courant max en A et RSHUNT en ohm, normalisation
  // Initialisation batterie 3
  Batterie3.begin();
  Batterie3.setMaxCurrentShunt(3, 0.0048, true); // Courant max en A et RSHUNT en ohm, normalisation

  // // initialisation de la ligne alert (pas utilisé, un exemple)
  // power_limit_W = 3;
  // limit = power_limit_W / (32 * (3 / 32768.0)); // puissance en valeur entiere (codé dans le composant)
  // Batterie1.setAlertLimit(limit);
  // Batterie1.setAlertRegister(INA236_POWER_OVER_LIMIT); // masque pour le mode de fonctionnement que l’on veut
}

void loop()
{
  envoi_int_CAN(1, BOOT_CARTE_PUISSANCE);

  // Lis les trames CAN reçu
  if (Can1.read(CAN_RX_msg)) // regarde si une trame à été reçu et met les informations dans l'objet CAN_RX_msg
  {
    // Serial.printf("ID %x\n", CAN_RX_msg.id); // affichage en hexa avec %x
    // Serial.printf("msg :");
    // for (i = 0; i < CAN_RX_msg.len; i++)
    // Serial.printf("%2x ", CAN_RX_msg.buf[i]);

    idTrames_CAN_Recu = CAN_RX_msg.id;
    switch (idTrames_CAN_Recu) // traite les messages CAN
    {
    case INTERRUPTEUR_BATT1:
      etat_interrupteur1 = CAN_RX_msg.buf[0] - 1; // ON si on a 1, off 0
      if (etat_interrupteur1)
        start_millis = millis();
      break;

    case INTERRUPTEUR_BATT2:
      etat_interrupteur2 = CAN_RX_msg.buf[0] - 1; // ON si on a 1, off 0
      if (etat_interrupteur2)
        start_millis = millis();
      break;

    case INTERRUPTEUR_BATT3:
      etat_interrupteur3 = CAN_RX_msg.buf[0] - 1; // ON si on a 1, off 0
      if (etat_interrupteur3)
        start_millis = millis();
      break;

    case MODE:
      // Serial.printf("mode recu ");
      mode_actuel = CAN_RX_msg.buf[0]; // 0 mode test, 1 mode match
      if (mode_actuel)
        start_millis = millis();
      break;

    case RPI:
      // Serial.printf("RPI recu ");
      etat_RPI = CAN_RX_msg.buf[0]; // ON si on a 1, off 0
      if (etat_RPI)
        start_millis = millis();
      break;

    default:
      break;
    }
  }

  val_aru = digitalRead(PINARU);
  envoi_int_CAN(val_aru, ARU);

  if (etat_RPI == 0 || etat_RPI == 2)
  {
    etat_interrupteur1 = OFF;
    etat_interrupteur2 = OFF;
    etat_interrupteur3 = OFF;
    step = 0;
    nbre_cellules_1 = 0;
    nbre_cellules_2 = 0;
    nbre_cellules_3 = 0;
    nbre_cellules_Main = 0;
  }
  else
  {
    if (val_aru == 1)
    { // eteint tout les interrupteurs
      etat_interrupteur1 = OFF;
      etat_interrupteur2 = OFF;
      etat_interrupteur3 = OFF;
      step = 0;
      nbre_cellules_1 = 0;
      nbre_cellules_2 = 0;
      nbre_cellules_3 = 0;
      nbre_cellules_Main = 0;
    }
    else
    {
      if (step < 10)
      {
        etat_interrupteur1 = ON;
        etat_interrupteur2 = ON;
        etat_interrupteur3 = ON;
      }
      else
      {
        VbattMAIN = BatterieMain.getBusVoltage();
        IbattMAIN = BatterieMain.getCurrent_mA();
        Vbatt1 = Batterie1.getBusVoltage();
        Ibatt1 = Batterie1.getCurrent_mA();
        Vbatt2 = Batterie2.getBusVoltage();
        Ibatt2 = Batterie2.getCurrent_mA();
        Vbatt3 = Batterie3.getBusVoltage();
        Ibatt3 = Batterie3.getCurrent_mA();

        // Calcul des nombre de cellules
        nbre_cellules_Main = calcul_nombre_cellules(VbattMAIN);
        nbre_cellules_1 = calcul_nombre_cellules(Vbatt1);
        nbre_cellules_2 = calcul_nombre_cellules(Vbatt2);
        nbre_cellules_3 = calcul_nombre_cellules(Vbatt3);
        // Calcul des tensions déchargé des batteries
        VbattMAIN_decharge = nbre_cellules_Main * TENSION_CELLULE_DECHARGE;
        Vbatt1_decharge = nbre_cellules_1 * TENSION_CELLULE_DECHARGE;
        Vbatt2_decharge = nbre_cellules_2 * TENSION_CELLULE_DECHARGE;
        Vbatt3_decharge = nbre_cellules_3 * TENSION_CELLULE_DECHARGE;
        // Calcul des tensions chargé des batteries
        VbattMAIN_charge = nbre_cellules_Main * TENSION_CELLULE_CHARGE;
        Vbatt1_charge = nbre_cellules_1 * TENSION_CELLULE_CHARGE;
        Vbatt2_charge = nbre_cellules_2 * TENSION_CELLULE_CHARGE;
        Vbatt3_charge = nbre_cellules_3 * TENSION_CELLULE_CHARGE;

        Vbatt1CAN = Vbatt1 * 100; // conversion pour envoyer en int (2 décimales)
        Vbatt2CAN = Vbatt2 * 100;
        Vbatt3CAN = Vbatt3 * 100;

        BattMAIN = Calcul_bat(VbattMAIN, VbattMAIN_charge, VbattMAIN_decharge);
        Batt1 = Calcul_bat(Vbatt1, Vbatt1_charge, Vbatt1_decharge);
        Batt2 = Calcul_bat(Vbatt2, Vbatt2_charge, Vbatt2_decharge);
        Batt3 = Calcul_bat(Vbatt3, Vbatt3_charge, Vbatt3_decharge);

        // Envoi des pourcentages de batterie sur le bus CAN
        envoi_int_CAN(Batt1, POURCENTAGE_BATT1);
        envoi_int_CAN(Batt2, POURCENTAGE_BATT2);
        envoi_int_CAN(Batt3, POURCENTAGE_BATT3);
        envoi_int_CAN(BattMAIN, BATT_MAIN);

        // gestion des décharge des batteries
        if (etat_interrupteur1 && Vbatt1 < 11.0)
        {
          envoi_int_CAN(1, ALERTE_DECHARGE_BATT1);
        }
        if (etat_interrupteur2 && Vbatt2 < 6.6)
        {
          envoi_int_CAN(1, ALERTE_DECHARGE_BATT2);
        }
        if (etat_interrupteur3 && Vbatt3 < 6.6)
        {
          envoi_int_CAN(1, ALERTE_DECHARGE_BATT3);
        }
        if (VbattMAIN < 11.2)
        {
          envoi_int_CAN(1, ALERTE_DECHARGE_MAIN);
        }
      }

      step += 1;
    }
  }

  // allume/eteint les interrupteurs
  digitalWrite(PININTERRUPTEURBATT1, etat_interrupteur1);
  digitalWrite(PININTERRUPTEURBATT2, etat_interrupteur2);
  digitalWrite(PININTERRUPTEURBATT3, etat_interrupteur3);

  /*
  // lecture des entrées
  // val_alert = digitalRead(PINALERT);

  unsigned long currentMillis = millis(); // Récupère le temps actuel

  /*
  // Vérifie si 5 secondes se sont écoulées
  if (currentMillis - previousMillis >= interval)
  {
    // Envoyer des tensions et courants messagex
    //envoi_tension_courant(VbattMAIN, IbattMAIN, BATT_MAIN);
    //envoi_tension_courant(Vbatt1, Ibatt1, BATT_1);
    //envoi_tension_courant(Vbatt2, Ibatt2, BATT_2);
    //envoi_tension_courant(Vbatt3, Ibatt3, BATT_3);
    //envoi_tension_min_max(Vbatt1,Vbatt1_decharge,Vbatt1_charge, BATT_1);

    // Envoi des nbre d'élements des batteries
    //CAN_TX_msg.id = CELLULE_BAT;
    //CAN_TX_msg.len = 1;
    //CAN_TX_msg.buf[0] = nbre_cellules_Main;
    //CAN_TX_msg.buf[1] = nbre_cellules_1;
    //CAN_TX_msg.buf[2] = nbre_cellules_2;
    //CAN_TX_msg.buf[3] = nbre_cellules_3;


    //envoi_float_CAN(Vbatt1,BATT_1);
    //envoi_float_CAN(Vbatt1_decharge, BATT_1_MIN);
    //envoi_float_CAN(Vbatt1_charge, BATT_1_MAX);

    //envoi_int_CAN(Vbatt1,BATT_1);
    envoi_int_CAN(Vbatt1CAN,BATT_1);
    //envoi_floatvir_CAN(Vbatt1,BATT_1);
    envoi_int_CAN(Vbatt1_decharge, BATT_1_MIN);
    envoi_int_CAN(Vbatt1_charge, BATT_1_MAX);
    envoi_int_CAN(mode_actuel, MODE);


    previousMillis = currentMillis; // Met à jour le temps de référence

  }


  // regarde si on a un court-circuit
  detectionCC(Vbatt1, Ibatt1, 1);
  detectionCC(Vbatt2, Ibatt2, 2);
  detectionCC(Vbatt3, Ibatt3, 3);

  valid_millis = millis();

  /*
  if(valid_millis - start_millis > interval)
  { // sécurité : coupe les batteries si elles restent allumées plus de 5 secondes sans commande
    if(etat_interrupteur1 && Vbatt1 < 6.5){
      etat_interrupteur1 = OFF;
    }
    if(etat_interrupteur2 && Vbatt2 < 11.3){
      etat_interrupteur2 = OFF;
    }
    if(etat_interrupteur3 && Vbatt3 < 6.5){
      etat_interrupteur3 = OFF;
    }
  }

  // Vérifie si 1 secondes se sont écoulées
  if (currentMillis - previousMillis >= interval)
  {

    /*
    envoi_int_CAN(Vbatt1CAN,BATT_1);
    envoi_int_CAN(Vbatt1_decharge, BATT_1_MIN);
    envoi_int_CAN(Vbatt1_charge, BATT_1_MAX);
    envoi_int_CAN(Vbatt2CAN,BATT_2);
    envoi_int_CAN(Vbatt2_decharge, BATT_2_MIN);
    envoi_int_CAN(Vbatt2_charge, BATT_2_MAX);
    envoi_int_CAN(Vbatt3CAN,BATT_3);
    envoi_int_CAN(Vbatt3_decharge, BATT_3_MIN);
    envoi_int_CAN(Vbatt3_charge, BATT_3_MAX);
    envoi_char_CAN(mode_actuel, MODE);


    // envoi_char_CAN(mode_actuel, MODE);

    previousMillis = currentMillis; // Met à jour le temps de référence
  }

  // Serial.printf("PININTERRUPTEURBATT1 %d | ", (int)etat_interrupteur1);
  // Serial.printf("PININTERRUPTEURBATT2 %d | ", (int)etat_interrupteur1);
  // Serial.printf("PININTERRUPTEURBATT2 %d | ", (int)etat_interrupteur1);
 */

  /*Serial.printf("aru = %1d", val_aru);
  Serial.printf(" | RPI = %1d", etat_RPI);
  // Batterie pricipale
  // Serial.printf(" | VbattMAIN:");
  // Serial.print(VbattMAIN);
  // Serial.printf("V | IbattMAIN:");
  // Serial.print(IbattMAIN);
  // Serial.printf(" | nbre_element_Main:%1d", nbre_cellulse_Main);
  // Serial.printf(" | VbattMain_decharge:");
  // Serial.print(VbattMAIN_decharge);


  // Serial.printf("V | mode =  ");
  // Serial.print(mode_actuel);
  // Serial.printf(" | Batt2  =  ");
  // Serial.print(Batt2);
  Serial.printf(" | Batt1 = %d%%", Batt1);
  Serial.printf(" | Batt2 = %d%%", Batt2);
  Serial.printf(" | Batt3 = %d%%", Batt3);
  Serial.printf(" | int1   = %d", etat_interrupteur1);
  Serial.printf(" | int2   = %d", etat_interrupteur2);
  Serial.printf(" | int3   = %d", etat_interrupteur3);
  //Serial.printf(" | Vbatt2   = %.1f", Vbatt2);
  // Serial.printf(" | Alerte decharge = %d", ALERTE_DECHARGE_BATT3);*/
  //Serial.printf(" | BattMAIN = %d%%", BattMAIN);
  //Serial.printf(" | VbattMAIN:");
  //Serial.print(VbattMAIN);
  // Serial.printf(" | VbattMAIN = %dV", VbattMAIN);
  printf("\n");
}

void conversion_float_to_4char(float nombre, unsigned char *adresse_tableau)
{
  float *ptr;
  ptr = (float *)adresse_tableau; // donne l'adresse du tableau au pointeur
  *ptr = nombre;                  // met à l'adresse du tableau la valeur réel (code binaire)
}

float conversion_4char_to_float(unsigned char *adresse_tableau)
{
  float nombre, *ptr;
  ptr = (float *)adresse_tableau; // donne l'adresse du tableau au pointeur qui pointe vers un float
  nombre = *ptr;                  // met dans le nombre le contenu à l'adresse du pointeur, le nombre réel codé en binaire
  return nombre;
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

/*
void envoi_tension_courant(float tension, float courant, int id)
{
  CAN_TX_msg.id = id;
  CAN_TX_msg.len = 8;
  char k;
  unsigned char tableau_V[4], tableau_I[4]; // tableau contenant les 32 bits des tensions et courants à envoyer
  // met la tension et courant dans les tableaux
  conversion_float_to_4char(tension, tableau_V);
  conversion_float_to_4char(courant, tableau_I);
  for (k = 0; k < 4; k++)
    CAN_TX_msg.buf[k] = tableau_V[k]; // met les 32 bits de la tension dans le message
  for (k = 0; k < 4; k++)
    CAN_TX_msg.buf[k + 4] = tableau_I[k]; // met les 32 bits du courant dans le message
  Can1.write(CAN_TX_msg);
}
*/
/*
void envoi_tension_min_max(float tension_actuelle, float tension_min, float tension_max, int id)
{
  CAN_TX_msg.id = id;
  CAN_TX_msg.len = 12; // 3 floats = 12 octets

  unsigned char tab_actuelle[4], tab_min[4], tab_max[4];

  // Convertit les floats en 4 octets
  conversion_float_to_4char(tension_actuelle, tab_actuelle);
  conversion_float_to_4char(tension_min, tab_min);
  conversion_float_to_4char(tension_max, tab_max);

  // Copie dans le buffer CAN
  for (int i = 0; i < 4; i++)
    CAN_TX_msg.buf[i] = tab_actuelle[i];
  for (int i = 0; i < 4; i++)
    CAN_TX_msg.buf[i + 4] = tab_min[i];
  for (int i = 0; i < 4; i++)
    CAN_TX_msg.buf[i + 8] = tab_max[i];

    // Envoie sur le bus
  Can1.write(CAN_TX_msg);
}
*/
/*
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
}*/

void envoi_int_CAN(int valeur, uint32_t id)
{
  CAN_TX_msg.id = id; // ID CAN pour le message
  CAN_TX_msg.len = 2; // 2 octets pour un int16_t (ou 4 pour un int32_t)
  // Pour un int16_t (2 octets) :
  CAN_TX_msg.buf[0] = valeur & 0xFF;        // Octet bas (LSB)
  CAN_TX_msg.buf[1] = (valeur >> 8) & 0xFF; // Octet haut(MSB)
  // Pour un int32_t (4 octets) :
  // CAN_TX_msg.buf[2] = (valeur >> 16) & 0xFF;
  // CAN_TX_msg.buf[3] = (valeur >> 24) & 0xFF;
  Can1.write(CAN_TX_msg);
}

void envoi_char_CAN(char valeur, uint32_t id)
{
  CAN_TX_msg.id = id; // ID CAN pour le message
  CAN_TX_msg.len = 1; // 1 octet pour un char

  CAN_TX_msg.buf[0] = valeur; // copie la valeur dans le buffer CAN
  Can1.write(CAN_TX_msg);     // Envoie le message
}

/*
void envoi_floatvir_CAN(float valeur, uint32_t id)
{
    // Convertit le float en 4 octets
    unsigned char *bytePtr = (unsigned char *)&valeur;

    CAN_TX_msg.id = id;  // ID CAN pour le message
    CAN_TX_msg.len = 4;  // 4 octets pour un float

    // Copie les 4 octets du float dans le buffer CAN
    for (int i = 0; i < 4; i++) {
        CAN_TX_msg.buf[i] = bytePtr[i];
    }

    Can1.write(CAN_TX_msg);  // Envoie le message
}*/

/*
// Fonction pour envoyer un float sur le bus CAN avec un ID donné
void envoi_float_CAN(float valeur, uint32_t id)
{
    unsigned char tab[4]; // Tableau pour stocker les 4 octets du float
    conversion_float_to_4char(valeur, tab); // Appel de ta fonction de conversion

    CAN_TX_msg.id = id;
    CAN_TX_msg.len = 4; // 4 octets pour un float

    // Copie les octets dans le buffer CAN
    for (int i = 0; i < 4; i++) {
        CAN_TX_msg.buf[i] = tab[i];
    }

    Can1.write(CAN_TX_msg); // Envoie le message
}*/

char Calcul_bat(float actuel, float charge, float decharge)
{
  int valeur_batterie;

  valeur_batterie = ((actuel - decharge) / (charge - decharge)) * 100;

  return (char)valeur_batterie;
}

void detectionCC(float tension, float courant, char voie_batterie)
{
  // tension en V et courant en mA
  // detecte un court circuit sur voie batterie 1, 2, 3
  // indique dans quelle voie on a un court-circuit
  // CC si on a du courant et peu de tension,
  // tension des batteries 7.2 V (6 elements), 12 V (10 elements)
  if ((tension < 4.8) && (courant > 500))
  {
    // y'a du courant et peu de tension > court-circuit
    switch (voie_batterie)
    {
    case 1: // voie 1
      CAN_TX_msg.id = COURT_CIRCUITBATT1;
      digitalWrite(PININTERRUPTEURBATT1, LOW); // coupe la voie 1
      etat_interrupteur1 = OFF;
      break;
    case 2: // voie 2
      CAN_TX_msg.id = COURT_CIRCUITBATT2;
      digitalWrite(PININTERRUPTEURBATT2, LOW); // coupe la voie 2
      etat_interrupteur2 = OFF;
      break;
    case 3: // voie 3
      CAN_TX_msg.id = COURT_CIRCUITBATT3;
      digitalWrite(PININTERRUPTEURBATT3, LOW); // coupe la voie 1
      etat_interrupteur3 = OFF;
      break;
    default:
      break;
    }
    CAN_TX_msg.len = 1;
    CAN_TX_msg.buf[0] = 1;
    Can1.write(CAN_TX_msg); // envoie dans le bus CAN que on a un CC
  }
}

// fonctions d'interruptions
void arret_urgence(void)
{ // fonction d'interruption pour l'arret d'urgence
  digitalWrite(PININTERRUPTEURBATT1, LOW);
  digitalWrite(PININTERRUPTEURBATT2, LOW);
  digitalWrite(PININTERRUPTEURBATT3, LOW);

  // éteint toutes les alimentations
  etat_interrupteur1 = OFF;
  etat_interrupteur2 = OFF;
  etat_interrupteur3 = OFF;

  // reinitialisation du timer
  start_millis = millis();
}
