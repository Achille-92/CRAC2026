#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <SoftwareSerial.h>
#include <HerkulexServo.h>
#include "SERVOS.h"
#include "STEPPER.h"
#include <STM32_CAN.h>

#define PAS_BAS 10
#define PAS_Retourner 700
#define PAS_HAUT 1150

STM32_CAN Can(CAN1, DEF); // Use PA11/12 pins for CAN1.

static CAN_message_t CAN_TX_msg;
static CAN_message_t CAN_RX_msg;

HerkulexServo servo_rotae(herkulex_bus, 0x02);
HerkulexServo servo_serer(herkulex_bus, 0x05);
TaskHandle_t _CAN_;

int tab_CAN[8];
int ID = 0;
int etat_rotae = 1;

int num_carte = 1, reboot = 1;
int etat_RPI = 0, etat_ESP_RPI = 0, on_pour_rpi = 0;
int action_a_faire = 0, sous_pince = 0, verif_action = 0, ack_action = 0, pas_act = 0, fin = 0;

int en_cours = 0, pre_action = 0;
static unsigned long lastSend = 0;
int temp = 0, E = 0, mouv = 0, verif_curseur = 0;
int action_suivante = 0; // Pour stocker la prochaine action

void reception(char ch);
// void CAN_(void *);
String serie;

void setup()
{
  Serial.begin(115200);
  Can.begin();
  Can.setBaudRate(500000); // 500KBPS

  Can.setFilter(0, 0x01, 0x1FFFFFFF);
  Can.setFilter(1, 0x502 + num_carte, 0x1FFFFFFF);
  Can.setFilter(2, 0x500 + num_carte, 0x1FFFFFFF);
  Can.setFilter(3, 0x504 + num_carte, 0x1FFFFFFF);
  Can.setFilter(6, 0x506, 0x1FFFFFFF);
  Can.setFilter(4, 0x206, 0x1FFFFFFF);
  Can.setFilter(5, 0x008, 0x1FFFFFFF);

  delay(5000);

  Serial.println("Test1");
  Serial.println("Tes2");
  temp = millis();
  vTaskStartScheduler();
  Serial.println("Insufficient RAM");
  while (1)
    ;
}

void loop()
{
  while (1)
  {
    vTaskDelay(1);

    // ── Lecture CAN ───────────────────────────────────────────
    if (Can.read(CAN_RX_msg))
    {
      if (CAN_RX_msg.id == 0x01)
      {
        etat_RPI = CAN_RX_msg.buf[0];
      }

      if ((CAN_RX_msg.id == 0x500 + num_carte))
      {
        int nouvelle_action = CAN_RX_msg.buf[0];

        // Si aucune action en cours (E == 0), accepter directement
        if (E == 0)
        {
          pre_action = action_a_faire;
          action_a_faire = nouvelle_action;
          // Serial.printf("Nouvelle action acceptée: %d\n", action_a_faire);
        }
        else
        {
          // Action en cours, stocker UNIQUEMENT si différente de l'action actuelle
          if (nouvelle_action != action_a_faire && nouvelle_action != 0)
          {
            action_suivante = nouvelle_action;
            // Serial.printf("Action en cours (%d), suivante stockée: %d\n", action_a_faire, action_suivante);
          }
        }
      }

      if ((CAN_RX_msg.id == 0x502 + num_carte))
      {
        sous_pince = CAN_RX_msg.buf[0];
      }

      if ((CAN_RX_msg.id == 0x504 + num_carte))
      {
        ack_action = CAN_RX_msg.buf[0];
        // Serial.printf("ACK CAN reçu: %d (action=%d, E=%d)\n", ack_action, action_a_faire, E);
      }

      if ((CAN_RX_msg.id == 0x206) && num_carte == 0)
      {
        mouv = CAN_RX_msg.buf[0];
      }
      if ((CAN_RX_msg.id == 0x506) && num_carte == 0)
      {
        fin = CAN_RX_msg.buf[0];
      }

      if ((CAN_RX_msg.id == 0x008) && num_carte == 0)
      {
        verif_curseur = CAN_RX_msg.buf[0];
      }
    }

    // ── Machine d'état RPI ────────────────────────────────────
    switch (etat_ESP_RPI)
    {
    case 0:
      on_pour_rpi = 1;
      if (etat_RPI == 1)
      {
        pas_act = 0;
        etat_ESP_RPI = 1;
        delay(500 * num_carte);
        initStepper();
        delay(1000);
        haut(abs(pas_act - (PAS_HAUT)));
        pas_act = PAS_HAUT;
        delay(8000);
        init_serial_1_for_herkulex();
        desserrer(12);
        delay(1000);
        rapprocher();
        delay(1000);
        tourner(12);
        delay(1200);
      }
      else
      {
        verif_action = 0;
        sous_pince = 0;
        action_a_faire = 0;
        action_suivante = 0;
        ack_action = 0;
        E = 0;
      }
      break;

    case 1:
      if ((etat_RPI == 0 || etat_RPI == 2))
      {
        etat_ESP_RPI = 0;
      }
      break;
    }

    // ── Envoi CAN périodique (toutes les 100 ms) ─────────────
    if (millis() - lastSend > 100)
    {
      lastSend = millis();

      CAN_TX_msg.id = 0x003 + num_carte;
      CAN_TX_msg.len = 1;
      CAN_TX_msg.buf[0] = on_pour_rpi;
      Can.write(CAN_TX_msg);

      CAN_TX_msg.id = 0x10C + num_carte;
      CAN_TX_msg.len = 1;
      CAN_TX_msg.buf[0] = verif_action;
      Can.write(CAN_TX_msg);
    }

    // ── Exécution des actions ─────────────────────────────────
    if ((etat_RPI == 1) && (mouv != 9 && mouv != 10) && fin == 0)
    {
      if (action_a_faire == 3)
        reboot = 0;
      // check_herkulex_errors_once();
      switch (action_a_faire)
      {
      case 0:
        if (reboot == 0)
        {
          init_serial_1_for_herkulex();
          delay(1000);
          rapprocher();
          delay(1000);
          tourner(12);
          delay(1200);
          desserrer(12);
          delay(1000);
          reboot = 1;
        }
        // Aucune action en cours, charger l'action suivante si disponible
        if (action_suivante != 0)
        {
          action_a_faire = action_suivante;
          action_suivante = 0;
          // Serial.printf("Chargement action suivante: %d\n", action_a_faire);
        }
        break;

      // ── ACTION 1 : ATTRAPER ──────────────────────────────
      case 1:
        switch (E)
        {
        case 0:
          // CRUCIAL: Reset ack_action au début de l'action
          ack_action = 0;
          verif_action = 0;
          CAN_TX_msg.id = 0x10C + num_carte;
          CAN_TX_msg.len = 1;
          CAN_TX_msg.buf[0] = verif_action;
          Can.write(CAN_TX_msg);
          // Serial.println("Action 1 - E0: Descente (ack_action reset)");
          bas(abs(pas_act - PAS_BAS));
          pas_act = PAS_BAS;
          E = 1;
          temp = millis();
          break;

        case 1:
          if ((millis() - temp) > 00)
          {
            //  Serial.printf("Action 1 - E1: Serrage pince %d\n", sous_pince);
            serrer(sous_pince);
            E = 2;
            temp = millis();
          }
          break;

        case 2:
          if ((millis() - temp) > 1500)
          {
            //  Serial.printf("Action 1 - E2: Montée + ACK (ack_action avant envoi=%d)\n", ack_action);
            verif_action = 1;
            CAN_TX_msg.id = 0x10C + num_carte;
            CAN_TX_msg.len = 1;
            CAN_TX_msg.buf[0] = verif_action;
            Can.write(CAN_TX_msg);

            haut(abs(pas_act - PAS_HAUT));
            pas_act = PAS_HAUT;
            maj_etat_pince(sous_pince, 1);

            E = 3;
            temp = millis();
          }
          break;

        case 3:
          if (ack_action == 2)
          {
            // Serial.println("Action 1 - E3: ACK confirmé, reset complet");
            verif_action = 0;
            sous_pince = 0;
            action_a_faire = 0;
            ack_action = 0;
            E = 0;
          }
          else
          {
            // Debug pour voir si on attend l'ACK
            static unsigned long dernierDebug = 0;
            if ((millis() - dernierDebug) > 1000)
            {
              // Serial.printf("Action 1 - E3: En attente ACK (ack_action=%d)\n", ack_action);
              dernierDebug = millis();
            }
          }
          break;
        }
        break;

      // ── ACTION 2 : RETOURNER ─────────────────────────────
      case 2:
        switch (E)
        {
        case 0:
          // CRUCIAL: Reset ack_action au début de l'action
          ack_action = 0;

          // Serial.println("Action 2 - E0: Envoi ACK immédiat (ack_action reset)");
          //  Envoi ACK immédiat
          verif_action = 1;
          CAN_TX_msg.id = 0x10C + num_carte;
          CAN_TX_msg.len = 1;
          CAN_TX_msg.buf[0] = verif_action;
          Can.write(CAN_TX_msg);

          E = 1;
          temp = millis();
          break;

        case 1:
          // Attente ACK de la RPI
          if (ack_action == 2)
          {
            // Serial.println("Action 2 - E1: ACK confirmé, début action");
            ack_action = 0;
            E = 2;
            temp = millis();
          }
          else
          {
            // Debug pour voir si on attend l'ACK
            static unsigned long dernierDebug = 0;
            if ((millis() - dernierDebug) > 1000)
            {
              // Serial.printf("Action 2 - E1: En attente ACK (ack_action=%d)\n", ack_action);
              dernierDebug = millis();
            }
          }
          break;

        case 2:
          if ((millis() - temp) > 00)
          {
            // Serial.println("Action 2 - E2: Ecarter + Descente");
            ecarter();
            bas(abs(pas_act - PAS_Retourner));
            pas_act = PAS_Retourner;
            E = 3;
            temp = millis();
          }
          break;

        case 3:
          if ((millis() - temp) > 00)
          {
            // Serial.printf("Action 2 - E3: Tourner pince %d\n", sous_pince);
            tourner(sous_pince);
            E = 4;
            temp = millis();
          }
          break;

        case 4:
          if ((millis() - temp) > 1500)
          {
            // Serial.println("Action 2 - E4: Montée + Rapprocher + Reset");
            haut(abs(pas_act - PAS_HAUT));
            pas_act = PAS_HAUT;
            rapprocher();

            verif_action = 0;
            sous_pince = 0;
            action_a_faire = 0;
            E = 0;
            temp = millis();
          }
          break;
        }
        break;

      // ── ACTION 3 : RELACHER ──────────────────────────────
      case 3:
        switch (E)
        {
        case 0:
          // CRUCIAL: Reset ack_action au début de l'action
          ack_action = 0;
          verif_action = 0;
          CAN_TX_msg.id = 0x10C + num_carte;
          CAN_TX_msg.len = 1;
          CAN_TX_msg.buf[0] = verif_action;
          Can.write(CAN_TX_msg);
          // Serial.println("Action 3 - E0: Descente (ack_action reset)");
          bas(abs(pas_act - PAS_BAS));
          pas_act = PAS_BAS;
          E = 1;
          temp = millis();
          break;

        case 1:
          if ((millis() - temp) > 00)
          {
            // Serial.printf("Action 3 - E1: Desserrer pince %d\n", sous_pince);
            desserrer(sous_pince);
            E = 2;
            temp = millis();
          }
          break;

        case 2:
          if ((millis() - temp) > 300)
          {
            // Serial.printf("Action 3 - E2: Montée + ACK + Reboot (ack_action avant envoi=%d)\n", ack_action);
            verif_action = 1;

            haut(abs(pas_act - PAS_HAUT));
            pas_act = PAS_HAUT;
            maj_etat_pince(sous_pince, 0);

            E = 3;
            temp = millis();
          }
          break;

        case 3:
          if (ack_action == 2)
          {
            // Serial.println("Action 3 - E3: ACK confirmé, reset complet");
            verif_action = 0;
            sous_pince = 0;
            action_a_faire = 0;
            ack_action = 0;
            E = 0;
          }
          else
          {
            // Debug pour voir si on attend l'ACK
            static unsigned long dernierDebug = 0;
            if ((millis() - dernierDebug) > 1000)
            {
              // Serial.printf("Action 3 - E3: En attente ACK (ack_action=%d)\n", ack_action);
              dernierDebug = millis();
            }
          }
          break;
        }
        break;

      default:
        break;
      }
    }
    else if (fin != 0)
    {
      desserrer(12);
      bas(pas_act);
    }
    else if ((mouv == 9 || mouv == 10) && num_carte == 0)
    {
      if (verif_curseur == 0)
      {
        curseur();
        bas(pas_act);
        pas_act = 0;
      }
      else if (verif_curseur != 0)
      {
        haut(abs(pas_act - PAS_HAUT));
        pas_act = PAS_HAUT;
        rapprocher();
      }
    }
    else if (verif_curseur != 0 && num_carte == 0 && (mouv == 9 || mouv == 10))
    {
      haut(abs(pas_act - PAS_HAUT));
      pas_act = PAS_HAUT;
      rapprocher();
    }
  }
}

void reception(char ch)
{
  static int i = 0;
  static String chaine = "";
  String commande;
  String valeur;
  int index, length;

  if ((ch == 13) or (ch == 10))
  {
    index = chaine.indexOf(' ');
    length = chaine.length();
    if (index == -1)
    {
      commande = chaine;
      valeur = "";
    }
    else
    {
      commande = chaine.substring(0, index);
      valeur = chaine.substring(index + 1, length);
    }

    if (commande == "ID")
    {
    }

    chaine = "";
  }
  else
  {
    chaine += ch;
  }
}