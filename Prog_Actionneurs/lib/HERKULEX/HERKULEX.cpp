#include <Arduino.h>
#include "HERKULEX.h"
#include <STM32FreeRTOS.h>

HardwareSerial Serial1(USART1);
HerkulexServoBus herkulex_bus(Serial1);
// Initialisation de la liaison série matérielle sur l'UART1

HerkulexServo my_servo(herkulex_bus, HERKULEX_BROADCAST_ID);
HerkulexServo Aimant_centre(herkulex_bus, SERVO_AIMANT_CENTRE);
HerkulexServo Aimant_gauche(herkulex_bus, SERVO_AIMANT_GAUCHE);
HerkulexServo Aimant_droit(herkulex_bus, SERVO_AIMANT_DROIT);
HerkulexServo Pivot_gauche(herkulex_bus, SERVO_PIVOT_GAUCHE);
HerkulexServo Pivot_droit(herkulex_bus, SERVO_PIVOT_DROIT);
HerkulexServo Pivot_pince(herkulex_bus, SERVO_PIVOT_PINCE);
HerkulexServo Pince(herkulex_bus, 0x06);

// Variables pour gérer l'intervalle de mise à jour
unsigned long last_update = 0; // Stocke le temps de la dernière mise à jour
unsigned long now = 0;         // Stocke le temps actuel
bool toggle = false;           // Booléen pour alterner entre deux positions
// Variables pour la position du servo
int pos, pos_angle;

void init_serial_1_for_herkulex()
{
  Serial1.setRx(PIN_SW_RX); // Associe la broche RX à l'UART1
  Serial1.setTx(PIN_SW_TX); // Associe la broche TX à l'UART1
  Serial1.begin(115200);    // Initialise la communication série à 115200 bauds
  my_servo.setTorqueOn();   // Active le couple du servo (mise sous tension)
}

void test_herkulex()
{
  herkulex_bus.update(); // Met à jour les servos connectés
  now = millis();        // Récupère le temps actuel en millisecondes

  // Commenté : récupération de la position du servo
  // pos = my_servo.getPosition();
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
      // my_servo.reboot();
      my_servo.setPosition(512 - 0 / 0.325, 50, HerkulexLed::Green);
      // Possibilité d'ajouter d'autres servos avec différentes positions
      // my_servo_2.setPosition(512-10/0.325, 50, HerkulexLed::Blue);
      // my_servo_3.setPosition(512+30/0.325, 50, HerkulexLed::Yellow);
    }
    else
    {
      // Déplace le servo à +45° en 50 cycles, allume la LED bleue
      // 512 + (45° / 0.325) = 650
      // 512 + 90°/0.325 = 789
      my_servo.setPosition(512 + 90 / 0.325, 100, HerkulexLed::Blue);
      my_servo.setTorqueOn();
      // my_servo_2.setPosition(512+10/0.325, 50, HerkulexLed::Blue);
      // my_servo_3.setPosition(512-30/0.325, 50, HerkulexLed::Yellow);
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
          Serial.print("0"); // Ajoute un "0" pour l'alignement des nombres
        }

        Serial.print(id, HEX);
      }
      else
      {
        Serial.print("--"); // Affiche "--" si aucun servo n'est détecté
      }
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
    return 0; // Retourne 0 si la détection est désactivée
  }
}

void aimant_cote_centre(void)
{
  // met le couple
  Pivot_gauche.setTorqueOn();
  Pivot_droit.setTorqueOn();

  // prepare le mouvement synchro
  herkulex_bus.prepareSynchronizedMove(50);

  SERVO_ROTATION.setPosition(512 + ANGLE_PIVOT_COTE_CENTRE / 0.325, 50, HerkulexLed::Blue); // +90 pour mettre au centre
  SERVO_ROTATION.setPosition(512 - ANGLE_PIVOT_COTE_CENTRE / 0.325, 50, HerkulexLed::Blue);  // -90 pour mettre au centre

  // execute le mouvement
  herkulex_bus.executeMove();
}

void aimant_cote_attraper(void)
{
  // met le couple
  Pivot_gauche.setTorqueOn();
  Pivot_droit.setTorqueOn();
  // prepare le mouvement synchroX

  Pivot_gauche.setPosition(512 + ANGLE_PIVOT_COTE_ATTRAPER / 0.325, 50, HerkulexLed::Green); // 0° pour poser
  Pivot_droit.setPosition(512 - ANGLE_PIVOT_COTE_ATTRAPER / 0.325, 50, HerkulexLed::Green);  // +90 pour poser
  // execute le mouvementX
}

void aimant_cote_ecarter(void)
{
  // met le couple
  Pivot_gauche.setTorqueOn();
  Pivot_droit.setTorqueOn();
  // prepare le mouvement synchro

  Pivot_gauche.setPosition(512 + ANGLE_PIVOT_COTE_ECARTER / 0.325, 50, HerkulexLed::Yellow); // -90° pour ecarter
  Pivot_droit.setPosition(512 - ANGLE_PIVOT_COTE_ECARTER / 0.325, 50, HerkulexLed::Yellow);  // +90 pour ecarter
  // execute le mouvement
}

void cmd_aimant_centre(bool mouvement)
{
  if (mouvement == RETIRER)
  {
    Aimant_centre.setTorqueOn();
    Aimant_centre.setPosition(512 + 45 / 0.325, 50);
  }
  if (mouvement == ATTRAPER)
  {
    Aimant_centre.setTorqueOn();
    Aimant_centre.setPosition(512 + 0 / 0.325, 50);
  }
}

void cmd_aimant_cote(char mouvement)
{
  Aimant_droit.setTorqueOn();
  Aimant_gauche.setTorqueOn();

  if (mouvement == ATTRAPER)
  {
    Aimant_droit.setPosition(512 + 0, 50);
    Aimant_gauche.setPosition(512 + 0, 50);
  }
  if (mouvement == RETIRER)
  {
    Aimant_droit.setPosition(512 + 45 / 0.325, 50);
    Aimant_gauche.setPosition(512 + -45 / 0.325, 50);
  }
}

void cmd_pivot_pompe(char mouvement)
{
  Pivot_pince.setTorqueOn();
  if (mouvement == DEPLOYER)
  {
    Pivot_pince.setPosition(512 + 10 / 0.325, 50); // angle final de 100
  }
  if (mouvement == RETRACTER)
  {
    Pivot_pince.setPosition(512 - 90 / 0.325, 50); // angle final de -90
  }
}

void cmd_pince(bool mouvement)
{
  Pince.setTorqueOn();
  if (mouvement == ATTRAPER)
  {
    Pince.setPosition(512 + ANGLE_PINCE_ATTRAPER / 0.325, 100);
  }
  if (mouvement == RETIRER)
  {
    Pince.setPosition(512 + ANGLE_PINCE_LACHER / 0.325, 100);
  }
}

// affiche la position en °
void display_servo_position(void)
{
  Serial.println(my_servo.readRam(HerkulexRamRegister::CalibratedPosition));
  Serial.print("Aimant_centre : ");
  Serial.print(Pivot_pince.getPosition());
  // Serial.print(0.325 * (Aimant_centre.getPosition() - 512));
  Serial.print(" | Aimant_gauche : ");
  Serial.print(0.325 * (Aimant_gauche.getPosition() - 512));
  Serial.print(" | Aimant_droit : ");
  Serial.print(0.325 * (Aimant_droit.getPosition() - 512));
  Serial.print(" | pivot_gauche : ");
  Serial.print(0.325 * (Pivot_gauche.getPosition() - 512));
  Serial.print(" | pivot_droit : ");
  Serial.print(0.325 * (Pivot_droit.getPosition() - 512));
  Serial.print(" | pivot_pince : ");
  Serial.print(0.325 * (Pivot_pince.getPosition() - 512));
  Serial.print(" | pince : ");
  Serial.println(0.325 * (Pince.getPosition() - 512));
}

// Allume la led en bleu pour les herkulex connectées
void test_connexion()
{
  // test pour voir lequel est connectée
  my_servo.setLedColor(HerkulexLed::Green); // allume la led des herkulex connectées
}

// si on veut la position d'un servo en particulier
int16_t get_servo_pos(HerkulexServo servo){
  return (servo.getPosition()-512) * 0.325; // on retrun la position du servo voulu
}

// donne la position de tout les servos en °, range tout des les variables
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
  *pos_servo_pivot_gauche  = (Pivot_gauche.getPosition() - 512) * 0.325;
  *pos_servo_pivot_droit   = (Pivot_droit.getPosition() - 512) * 0.325;
  *pos_servo_aimant_droit  = (Aimant_droit.getPosition() - 512) * 0.325;
  *pos_servo_aimant_gauche = (Aimant_gauche.getPosition() - 512) * 0.325;
  *pos_servo_aimant_centre = (Aimant_centre.getPosition() - 512) * 0.325;
  *pos_servo_pince = (Pince.getPosition() - 512) * 0.325;
  *pos_servo_pivot_pince   = (Pivot_pince.getPosition() - 512) * 0.325;
}

void restart_all_servo(void){
  my_servo.reboot();
  vTaskDelay(pdMS_TO_TICKS(225));
}

void change_id(uint8_t id, HerkulexServo old_, HerkulexServo new_){

  old_.setLedColor(HerkulexLed::Blue);
  delay(10000);
  old_.reboot();
  delay(500); // OK
  // lis pour lever temporairement la protection de la rom
  Serial.printf("%d",old_.readEep(HerkulexEepRegister::ID));
  old_.writeEep(HerkulexEepRegister::ID, id);
  Serial.printf("%d",old_.readEep(HerkulexEepRegister::ID));
  delay(300); // un peu plus long
  new_.reboot();
  delay(300); // pour être certain
  new_.setLedColor(HerkulexLed::White);
  while(1){
    if (Serial.available() > 0)
    {
      char c = Serial.read();

      if (c == 'f') break;

    }
  }
}