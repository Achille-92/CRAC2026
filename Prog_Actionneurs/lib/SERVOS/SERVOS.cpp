#include <Arduino.h>
#include "SERVOS.h"
#include <STM32FreeRTOS.h>

HardwareSerial Serial1(USART1);
HerkulexServoBus herkulex_bus(Serial1);
// Initialisation de la liaison série matérielle sur l'UART1

HerkulexServo all_servo(herkulex_bus, HERKULEX_BROADCAST_ID);

// pince arrière 1
HerkulexServo servo_attraper_interieur(herkulex_bus, 0x0B);
HerkulexServo servo_retourner_interieur(herkulex_bus, 0x02);
HerkulexServo servo_retourner_exterieur(herkulex_bus, 0x03);
HerkulexServo servo_attraper_exterieur(herkulex_bus, 0x04);
HerkulexServo servo_ecarter_pince(herkulex_bus, 0x05);
// pince avant 0
/*HerkulexServo servo_attraper_interieur(herkulex_bus, 0x06);
HerkulexServo servo_retourner_interieur(herkulex_bus, 0x07);
HerkulexServo servo_retourner_exterieur(herkulex_bus, 0x09);
HerkulexServo servo_attraper_exterieur(herkulex_bus, 0x08);
HerkulexServo servo_ecarter_pince(herkulex_bus, 0x0A);*/
// Variables d'état des pinces (1 = objet présent, 0 = vide)
int objet_pince_int = 0;
int objet_pince_ext = 0;

// Tableau des servos surveillés par check_herkulex_errors_once
static HerkulexServo* servos_surveilles[] = {
  &servo_attraper_interieur,
  &servo_attraper_exterieur,
  &servo_retourner_interieur,
  &servo_retourner_exterieur,
  &servo_ecarter_pince
};
static const char* noms_servos[] = {
  "ATT_INT", "ATT_EXT", "ROT_INT", "ROT_EXT", "ECART"
};
static const uint8_t NB_SERVOS = 5;

// Variables pour gérer l'intervalle de mise à jour
unsigned long last_update = 0; // Stocke le temps de la dernière mise à jour
unsigned long now         = 0; // Stocke le temps actuel
bool toggle               = false; // Booléen pour alterner entre deux positions
char etat_rota            = 0; // État actuel de la rotation du servo (0 -> 0°, 1 -> 90°, 2 -> 180°)

// Variables pour la position du servo
int pos, pos_angle;

// ─────────────────────────────────────────────────────────────
// Initialisation du bus Herkulex.
// Envoie la position de repos immédiatement après setTorqueOn()
// pour écraser la goal position stockée avant le reboot et
// éviter que le servo parte vers une position indéterminée.
// ─────────────────────────────────────────────────────────────
void init_serial_1_for_herkulex()
{
  Serial1.setRx(PIN_SW_RX); // Associe la broche RX à l'UART1
  Serial1.setTx(PIN_SW_TX); // Associe la broche TX à l'UART1
  Serial1.begin(115200);    // Initialise la communication série à 115200 bauds

  // Pour chaque servo : torque ON puis position de repos immédiate
  // pour écraser la goal position stockée avant le reboot
  // et éviter que le servo parte vers une position indéterminée
  servo_attraper_interieur.reboot();
  delay(300);
  servo_attraper_interieur.setTorqueOn();
  servo_attraper_interieur.setLedColor(HerkulexLed::Blue);
  delay(100);
servo_attraper_exterieur.reboot();
  delay(300);
  servo_attraper_exterieur.setTorqueOn();
  servo_attraper_exterieur.setLedColor(HerkulexLed::Blue);
  delay(100);
servo_retourner_interieur.reboot();
  delay(300);
  servo_retourner_interieur.setTorqueOn();
  servo_retourner_interieur.setLedColor(HerkulexLed::Green);
  delay(100);
servo_retourner_exterieur.reboot();
  delay(300);
  servo_retourner_exterieur.setTorqueOn();
  servo_retourner_exterieur.setLedColor(HerkulexLed::Green);
  delay(100);
servo_ecarter_pince.reboot();
  delay(300);
  servo_ecarter_pince.setTorqueOn();
  servo_ecarter_pince.setLedColor( HerkulexLed::Cyan);

  // Laisse le temps aux servos d'atteindre leur position de repos
  // avant que la suite du code envoie d'autres commandes
  delay(1200);
}

// ─────────────────────────────────────────────────────────────
// Met à jour l'état des pinces selon l'action effectuée.
// pince : 1 = intérieure, 2 = extérieure, 12 = les deux
// etat  : 1 = objet présent, 0 = vide
// ─────────────────────────────────────────────────────────────
void maj_etat_pince(int pince, int etat)
{
  if (pince == 1 || pince == 12) objet_pince_int = etat;
  if (pince == 2 || pince == 12) objet_pince_ext = etat;
  //Serial.printf("[PINCE] int=%d  ext=%d\n", objet_pince_int, objet_pince_ext);
}

// ─────────────────────────────────────────────────────────────
// Vérifie tous les servos UNE seule fois.
// À appeler uniquement après restart_all_servo() car le bus
// vient d'être réinitialisé : pas de risque de corruption.
// ─────────────────────────────────────────────────────────────
void check_herkulex_errors_once(void)
{
  for (uint8_t i = 0; i < NB_SERVOS; i++)
  { HerkulexStatusError err;
    HerkulexStatusDetail det;
     servos_surveilles[i]->getStatus(err,det);
   

    if (err != HerkulexStatusError::None)
    {
      //Serial.printf("[HERKULEX] Erreur servo %s : 0x%02X -> reboot\n",
       //             noms_servos[i], status_raw);

      // Diagnostic détaillé
      //if ((err & HerkulexStatusError::Overload)         != HerkulexStatusError::None)
      //  Serial.println("  -> Surcharge");
      //if ((err & HerkulexStatusError::TemperatureLimit) != HerkulexStatusError::None)
      //  Serial.println("  -> Temperature excessive");
      //if ((err & HerkulexStatusError::InputVoltage)     != HerkulexStatusError::None)
     //   Serial.println("  -> Tension hors limites");
      //if ((err & HerkulexStatusError::DriverFault)      != HerkulexStatusError::None)
//Serial.println("  -> Defaut driver");

      // Reboot du servo fautif : safe car init_serial va suivre
      servos_surveilles[i]->reboot();
      delay(300);
      servos_surveilles[i]->setTorqueOn();
    }
    else
    {
    //  Serial.printf("[HERKULEX] Servo %s OK\n", noms_servos[i]);
    }
  }
}

// ─────────────────────────────────────────────────────────────
// Actions servo — fire and forget.
// Envoie la commande et retourne immédiatement.
// Les délais entre étapes sont gérés dans le main.
// ─────────────────────────────────────────────────────────────
void tourner(int pince)
{
  static int etat_rotae = 1;
  int position;
  switch (etat_rotae)
  {
  case 0: position = position_defaut;    etat_rotae = 1; break;
  case 1: position = position_retourner; etat_rotae = 0; break;
  }
  if (pince == 1 || pince == 12)
    servo_retourner_interieur.setPosition(position, 100, HerkulexLed::Blue);
  if (pince == 2 || pince == 12)
    servo_retourner_exterieur.setPosition(position, 100, HerkulexLed::Green);
}

void serrer(int pince)
{
  if (pince == 1 || pince == 12)
    servo_attraper_interieur.setPosition(position_attraper_interieur, 100, HerkulexLed::Green);
  if (pince == 2 || pince == 12)
    servo_attraper_exterieur.setPosition(position_attraper, 100, HerkulexLed::Green);
}

void desserrer(int pince)
{
  if (pince == 1 || pince == 12)
    servo_attraper_interieur.setPosition(position_ecarter, 100, HerkulexLed::Blue);
  if (pince == 2 || pince == 12)
    servo_attraper_exterieur.setPosition(position_ecarter, 100, HerkulexLed::Green);
}

void ecarter(void)
{
  servo_ecarter_pince.setPosition(position_ecarter_pince, 100, HerkulexLed::Green);
  herkulex_bus.executeMove();
}

void rapprocher(void)
{
  servo_ecarter_pince.setPosition(position_rapprocher, 40, HerkulexLed::Blue);
  //herkulex_bus.executeMove();
}

void curseur(void)
{
  // Position neutre du servo écarteur utilisée pour l'action curseur
  servo_ecarter_pince.setPosition(position_curseur, 40, HerkulexLed::Yellow);
  //herkulex_bus.executeMove();
}

void test_herkulex()
{
  herkulex_bus.update(); // Met à jour les servos connectés
  now = millis();        // Récupère le temps actuel en millisecondes

  // Commenté : récupération de la position du servo
  // pos = all_servo.getPosition();
  // pos_angle = (pos-512)*0.325;  // Conversion en degrés
  // Serial.printf("pos : %4d | %4d °\n", pos, pos_angle);

  // Si 5000 ms (5 secondes) se sont écoulées depuis la dernière mise à jour
  if ((now - last_update) > 5000)
  {
    // called every 1000 ms
    if (toggle)
    {
      // Déplace le servo à -90° en 50 cycles, allume la LED verte
      // 512 - 90°/0.325 = 235
      // all_servo.reboot();
      all_servo.setPosition(512 - 0 / 0.325, 50, HerkulexLed::Green);
    }
    else
    {
      // Déplace le servo à +90° en 100 cycles, allume la LED bleue
      // 512 + (45° / 0.325) = 650
      // 512 + 90°/0.325 = 789
      all_servo.setPosition(512 + 90 / 0.325, 100, HerkulexLed::Blue);
      all_servo.setTorqueOn();
    }
    last_update = now; // Met à jour le dernier temps d'exécution
    toggle = !toggle;  // Alterne entre les deux positions
  }
}

int detect_id(bool activate)
{
  if (activate)
  {
    herkulex_bus.update(); // Met à jour les servos sur le bus

    uint8_t servos_found = 0;
    // Boucle sur tous les ID possibles (0x00 à 0xFD)
    for (uint8_t id = 0; id <= 0xFD; id++)
    {
      HerkulexPacket resp; // Stocke la réponse du servo
      bool success = herkulex_bus.sendPacketAndReadResponse(resp, id, HerkulexCommand::Stat);
      if (success)
      {
        servos_found++; // Incrémente le compteur si un servo est trouvé
        // Affichage de l'ID au format hexadécimal
        if (id <= 0x0F)
        {
          Serial.print("0");
        } // Ajoute un "0" pour l'alignement des nombres
        Serial.print(id, HEX);
      }
      else
      {
        Serial.print("--");
      } // Affiche "--" si aucun servo n'est détecté
      // Saut de ligne toutes les 15 adresses affichées
      if (((id + 1) % 0x0F) == 0)
      {
        Serial.println();
      }
      else
      {
        Serial.print(" ");
      }
    }
    // Affichage du nombre total de servos trouvés
    Serial.println();
    Serial.println("Done!");
    Serial.print("Found ");
    Serial.print(servos_found);
    Serial.println(" servos.");

    return 0xFD; // Retourne l'adresse de diffusion (broadcast ID)
  }
  else
  {
    return 0;
  } // Retourne 0 si la détection est désactivée
}

// affiche la position en °
void display_servo_position(void)
{
  Serial.println(all_servo.readRam(HerkulexRamRegister::CalibratedPosition));
  Serial.print("Aimant_centre : ");
  // Serial.print(Pivot_pince.getPosition());
}

// Allume la led en vert pour les herkulex connectées
void test_connexion()
{
  // test pour voir lequel est connectée
  all_servo.setLedColor(HerkulexLed::Green); // allume la led des herkulex connectées
}

// si on veut la position d'un servo en particulier
int16_t get_servo_pos(HerkulexServo servo)
{
  return (servo.getPosition() - 512) * 0.325; // on retourne la position du servo voulu
}

// donne la position de tous les servos en °, range tout dans les variables
void get_all_servo_pos(
    short *pos_servo_pivot_gauche,
    short *pos_servo_pivot_droit,
    short *pos_servo_aimant_droit,
    short *pos_servo_aimant_gauche,
    short *pos_servo_aimant_centre,
    short *pos_servo_pince,
    short *pos_servo_pivot_pince)
{
  // range les pos des servos en ° dans les variables
  /**pos_servo_pivot_gauche  = (Pivot_gauche.getPosition() - 512) * 0.325;
   *pos_servo_pivot_droit   = (Pivot_droit.getPosition() - 512) * 0.325;
   *pos_servo_aimant_droit  = (Aimant_droit.getPosition() - 512) * 0.325;
   *pos_servo_aimant_gauche = (Aimant_gauche.getPosition() - 512) * 0.325;
   *pos_servo_aimant_centre = (Aimant_centre.getPosition() - 512) * 0.325;
   *pos_servo_pince = (Pince.getPosition() - 512) * 0.325;
   *pos_servo_pivot_pince   = (Pivot_pince.getPosition() - 512) * 0.325;*/
}

void restart_all_servo(void)
{
  all_servo.reboot();
  vTaskDelay(pdMS_TO_TICKS(225));
}

void change_id(uint8_t id, HerkulexServo old_, HerkulexServo new_)
{
  old_.setLedColor(HerkulexLed::Blue);
  delay(10000);
  Serial.print(1);
  old_.reboot();
  delay(500); // OK
  Serial.print(1);
  // lis pour lever temporairement la protection de la rom
  Serial.printf("%d", old_.readEep(HerkulexEepRegister::ID));
  Serial.print(1);
  old_.writeEep(HerkulexEepRegister::ID, id);
  // Serial.print(1);
  Serial.printf("%d", old_.readEep(HerkulexEepRegister::ID));
  // Serial.print(1);
  delay(300); // un peu plus long
  // Serial.print(1);
  new_.reboot();
  delay(300); // pour être certain
  // Serial.print(1);
  new_.setLedColor(HerkulexLed::White);
  while (1)
  {
    if (Serial.available() > 0)
    {
      char c = Serial.read();
      if (c == 'f') break;
    }
  }
}