#include <Arduino.h>
#include <Wire.h>
#include <DualVNH5019MotorShield.h>
#include <ESP32Encoder.h>
#include <BluetoothSerial.h>
#include <CAN.h>
#include <PID.h>
#include <utility.h>
#include <Polaire.h>
#include <Vitesse.h>
#include "esp_timer.h"

#define TX_GPIO_NUM 5
#define RX_GPIO_NUM 4
#define X 0
#define Y 1
#define bleu 2
#define jaune 3
#define tic_compensation_gauche 1.0f
#define tic_compensation_droit 1.0f
void Asserv(void *);
void CAN_(void *);
void recalage(int type);
void aquisition(void);
void reception(char ch);
void updateEncoders(void);
void curseur(int type);
float limit_pwm(float pwm, float speed);

struct coo
{
  float x;
  float y;
};
coo liste_point[10] = {0, 0};

int tic_gauche = 0, tic_droit = 0;
float tour_gauche = 0, tour_droit = 0;
float distance_droit = 0, old_dist_droit = 0, diff_dist_droit = 0;
float distance_gauche = 0, old_dist_gauche = 0, diff_dist_gauche = 0;
float dist = 0, teta = 0, delta_x = 0, delta_y = 0;
float teta_act = 0, diff_teta = 0, teta_cons = PI;
float pos_act[2] = {-100, -100};
float pos_fin[2] = {275, 1650};
float pos_traj[2] = {0, 0};
float E = 0, off_teta_act = 0, pre_mouv = 0;

float coef_MOT[21] = {1.0f, 1.0f, 1.0f, 1.0f, 1.1f, 1.14f, 1.0f, 1.1f, 1.15f, 1.0f, 0.0f, 1.0f, 1.15f, 1.1f, 1.14f, 1.1f, 1.1f, 1.0f, 1.0f, 1.0f, 1.0f}; //-400 -360 -320 -280 -240 -200 -160 -120 -80 -40 0 40 80 120 160 200 240 280 320 360 400

const float tic_tour_droit = 3824.0, tic_tour_gauche = 3823.0, Rayon = 11.235;

int packetSize = 0;
float Vg = 0, Vd = 0, PWM_G = 0, PWM_D = 0, Can_temp = 0;
float PWM_Dpre = 0, PWM_Gpre = 0;
int old_mouv = 0, acuser = 0, acuser_angle = 0, acuser_recalage = 0;
int pos_temp[2] = {0, 0}, err_odo = 0, verif_X = 0, verif_Y = 0;

float moy_vitG[6] = {0, 0, 0, 0, 0, 0};
float moy_vitD[6] = {0, 0, 0, 0, 0, 0};

float teta_recu = -90.0f, teta_cons_recu = 0;
int etat_ESP_RPI = 0, etat_RPI = 0, Etat_ESP = 1;
static bool x_ok = false, y_ok = false, angle_ok = false;
bool mouvement = false;
int nbr_point = 0, mouv = 0, RPI = 1;
bool trajectoire_recue = false;

float verif_mouv = 0, verif_angle = 0, verif_recalage = 0;
int etape = 0;
int delta = 0;
float temp = 0, vcons = 0, tempo = 0, te = 0;

int k = 0, stop = 0, tt = 0, col_bleu = 0, col_jaune = 0;
float vlin = 0, vang = 0, vang2 = 0;
float sendlast = 0, odo = 0;
int calX = 0, calY = 0;
float vitese = 0, last_calc = 0;
float diff_d_co = 0, old_d_co = 0;
float diff_g_co = 0, old_g_co = 0;
float teta2 = 0, delta_x2 = 0, delta_y2 = 0;
int curseur_etape = 0, temp_curseur, verif_curseur = 0;

// Variables internes encodeur droit
int64_t lastRaw_droit = 0;
int accumPos_droit = 0;
int accumNeg_droit = 0;

// Variables internes encodeur gauche
int64_t lastRaw_gauche = 0;
int accumPos_gauche = 0;
int accumNeg_gauche = 0;

// --- Anti-patinage ---
static unsigned long slip_start_ms = 0;
static bool slip_active = false;
static float slip_origin_x = 0, slip_origin_y = 0;
static int slip_direction = 0; // +1 ou -1
const float PWM_SLIP_THRESHOLD = 100.0f;
const float SPEED_SLIP_THRESHOLD = 0.003f;
const unsigned long SLIP_DETECT_DELAY = 300;
const float SLIP_DISTANCE = 150.0f;

DualVNH5019MotorShield md;
ESP32Encoder encoder;
ESP32Encoder encoder2;
TaskHandle_t Asservissement;
TaskHandle_t _CAN_;

Polaire position;
Polaire angle2;
vitesse moteur;

// ─────────────────────────────────────────────────────────────────────────
void setup()
{
  Serial.begin(115200);

  md.init();
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachFullQuad(39, 36);
  encoder2.attachFullQuad(22, 23);
  encoder.setFilter(1023); // valeur maximale = 12.8µs
  encoder2.setFilter(1023);
  encoder.setCount(0);
  encoder2.setCount(0);
  md.setSpeeds(0, 0);

  Serial.println("Encoder Start = " + String((int32_t)encoder.getCount()));

  if (RPI == 1)
  {
    CAN.setPins(RX_GPIO_NUM, TX_GPIO_NUM);
    if (!CAN.begin(1000E3))
    {
      Serial.println("Starting CAN failed!");
      while (1)
        ;
    }
  }

  moteur.reset();
  tempo = millis();

  xTaskCreate(Asserv, "Asserv", 32000, NULL, 15, &Asservissement);
  xTaskCreate(CAN_, "CAN", 16000, NULL, 13, &_CAN_);
}

void loop() {}

// ─────────────────────────────────────────────────────────────────────────
void CAN_(void *)
{
  while (1)
  {
    if (RPI == 1)
    {
      int packetSize = CAN.parsePacket();
      if (packetSize)
      {
        uint32_t canId = CAN.packetId();
        if (CAN.available())
        {

          if (canId == 0x01)
            etat_RPI = CAN.read();

          // pos CAN acceptee seulement pendant l'init (etat_ESP_RPI == 0)
          if (canId == 0x200)
          {
            CAN.readBytes((uint8_t *)&Can_temp, packetSize);
            if (Can_temp > 50 && Can_temp < 2950 && etat_ESP_RPI == 0)
            {
              encoder2.setCount(0);
              encoder.setCount(0);
              diff_dist_droit = 0;
              diff_dist_gauche = 0;
              distance_droit = 0;
              distance_gauche = 0;
              tour_droit = 0;
              tour_gauche = 0;
              tic_droit = 0;
              tic_gauche = 0;
              old_d_co = 0;
              old_g_co = 0;
              diff_d_co = 0;
              diff_g_co = 0;
              old_dist_droit = 0;
              old_dist_gauche = 0;
              off_teta_act = 0;
              pos_act[0] = Can_temp;
            }
          }

          if (canId == 0x201)
          {
            CAN.readBytes((uint8_t *)&Can_temp, packetSize);
            if (Can_temp > 50 && Can_temp < 1950 && etat_ESP_RPI == 0)
              pos_act[1] = Can_temp;
          }

          if (canId == 0x202)
          {
            float tmp_teta;
            CAN.readBytes((uint8_t *)&tmp_teta, packetSize);
            teta_recu = tmp_teta;
          }

          if (canId == 0x209)
          {
            CAN.readBytes((uint8_t *)&Can_temp, packetSize);
            if (Can_temp > 50 && Can_temp < 2950)
              liste_point[0].x = Can_temp;
          }

          if (canId == 0x20A)
          {
            CAN.readBytes((uint8_t *)&Can_temp, packetSize);
            if (Can_temp > 50 && Can_temp < 1950)
              liste_point[0].y = Can_temp;
          }

          if (canId == 0x203)
          {
            CAN.readBytes((uint8_t *)&Can_temp, packetSize);
            if (Can_temp > 50 && Can_temp < 2950)
              liste_point[1].x = Can_temp;
          }

          if (canId == 0x204)
          {
            CAN.readBytes((uint8_t *)&Can_temp, packetSize);
            if (Can_temp > 50 && Can_temp < 1950)
              liste_point[1].y = Can_temp;
          }

          if (canId == 0x205)
          {
            float tmp_cons;
            CAN.readBytes((uint8_t *)&tmp_cons, packetSize);
            tmp_cons -= 360;
            teta_cons_recu = tmp_cons;
            teta_cons = teta_cons_recu * PI / 180.0f;
          }

          if (canId == 0x206)
          {
            int tmp_mouv = CAN.read();
            Serial.print("mouv : ");
            Serial.println(tmp_mouv);
            pre_mouv = mouv;
            mouv = tmp_mouv;
            if (pre_mouv == 4 && mouv == 1 && calX == 1 && calY == 1)
            {
              odo = millis();
              calX = 0;
              calY = 0;
              err_odo = 1;
            }
          }

          if (canId == 0x207)
          {
            int tmp;
            CAN.readBytes((uint8_t *)&tmp, packetSize);
            acuser = tmp;
          }

          if (canId == 0x208)
          {
            int tmp;
            CAN.readBytes((uint8_t *)&tmp, packetSize);
            acuser_angle = tmp;
          }
          if (canId == 0x20B)
          {
            int tmp;
            CAN.readBytes((uint8_t *)&tmp, packetSize);
            acuser_recalage = tmp;
          }

          if (canId == 0x20C)
          {
            int tmp;
            CAN.readBytes((uint8_t *)&tmp, packetSize);
            acuser_recalage = tmp;
          }
        }
      }

      switch (etat_ESP_RPI)
      {
      case 0:
        mouvement = false;
        if (pos_act[0] != -1)
          x_ok = true;
        if (pos_act[1] != -1)
          y_ok = true;
        if (teta_recu != -181)
          angle_ok = true;

        if (x_ok && y_ok && angle_ok)
        {
          if (etat_RPI == 1)
          {
            etat_ESP_RPI = 1;
            encoder.setCount(0);
            encoder2.setCount(0);
            distance_droit = 0;
            distance_gauche = 0;
            old_d_co = 0;
            old_g_co = 0;
            diff_d_co = 0;
            diff_g_co = 0;
            verif_mouv = 0;
            verif_angle = 0;
            tt = millis();
          }
        }
        break;

      case 1:
      {
        mouvement = true;

        if (etat_RPI == 0 || etat_RPI == 2)
        {
          etat_ESP_RPI = 0;
          x_ok = false;
          y_ok = false;
          angle_ok = false;
          mouv = 0;
          PWM_G = 0;
          PWM_D = 0;
        }

        float teta_send = (180.0f * limit_angle(teta_act)) / PI;

        static unsigned long lastSend = 0;
        if (millis() - lastSend > 20)
        {
          lastSend = millis();

          CAN.beginPacket(0x100);
          CAN.write((uint8_t *)&pos_act[0], sizeof(float));
          CAN.endPacket();

          CAN.beginPacket(0x101);
          CAN.write((uint8_t *)&pos_act[1], sizeof(float));
          CAN.endPacket();

          CAN.beginPacket(0x102);
          CAN.write((uint8_t *)&teta_send, sizeof(float));
          CAN.endPacket();

          CAN.beginPacket(0x10F);
          CAN.write((uint8_t *)&verif_angle, sizeof(float));
          CAN.endPacket();
        }
        break;
      }
      }

      if (millis() - sendlast > 100)
      {
        sendlast = millis();

        CAN.beginPacket(0x02);
        CAN.write((uint8_t *)&Etat_ESP, sizeof(float));
        CAN.endPacket();

        CAN.beginPacket(0x110);
        CAN.write((uint8_t *)&verif_recalage, sizeof(float));
        CAN.endPacket();

        CAN.beginPacket(0x10E);
        CAN.write((uint8_t *)&verif_mouv, sizeof(float));
        CAN.endPacket();

        CAN.beginPacket(0x008);
        CAN.write((uint8_t *)&verif_curseur, sizeof(float));
        CAN.endPacket();

        CAN.beginPacket(0x111);
        CAN.write((uint8_t *)&verif_recalage, sizeof(float));
        CAN.endPacket();

        CAN.beginPacket(0x112);
        CAN.write((uint8_t *)&verif_recalage, sizeof(float));
        CAN.endPacket();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ─────────────────────────────────────────────────────────────────────────
void Asserv(void *)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    if (err_odo == 0)
    {
      odo = millis();
    }
    if (col_bleu == 1)
    {
      if ((millis() - odo > 10000) && err_odo == 2)
      {
        pos_act[1] -= 25;
        pos_act[0] -= 10;
        err_odo = 3;
        odo = millis();
      }
      if ((millis() - odo > 10000) && err_odo == 3)
      {

        // pos_act[0] += 10;
        err_odo = 4;
        odo = millis();
      }
    }
    /*if (col_jaune == 1)
    {
      if ((millis() - odo > 10000) && err_odo == 2)
      {
        pos_act[1] -= 10;
        pos_act[0] += 7;
        err_odo = 3;
        odo = millis();
      }
      if ((millis() - odo > 10000) && err_odo == 3)
      {
        pos_act[1] -= 10;
        //pos_act[0] += 7;
        err_odo = 4;
        odo = millis();
      }
    }*/

    int local_mouv = mouv;

    if (local_mouv == 0)
      moteur.reset();
    if (liste_point[0].x != 0 && liste_point[0].y != 0)
    {

      if (verif_mouv >= 1)
      {
        pos_fin[0] = liste_point[0].x;
        pos_fin[1] = liste_point[0].y;
      }
      else
      {
        pos_fin[0] = liste_point[0].x;
        pos_fin[1] = liste_point[0].y;
      }

      if (local_mouv == 5 || local_mouv == 6)
      {
        if (etat_RPI == 1 &&
            comparer(pos_act[0], liste_point[0].x, 30) &&
            comparer(pos_act[1], liste_point[0].y, 30))
          verif_mouv = 1;
        else
          verif_mouv = 0;
      }
      else if (local_mouv == 1 || local_mouv == 2)
      {
        if (etat_RPI == 1 &&
            comparer(pos_act[0], liste_point[0].x, 70) &&
            comparer(pos_act[1], liste_point[0].y, 70))
        {
          verif_mouv = 1;
          if (comparer(pos_act[0], liste_point[0].x, 30) &&
              comparer(pos_act[1], liste_point[0].y, 30))
            verif_mouv = 1;
        }
        else
        {
          verif_mouv = 0;
        }
      }
    }
    aquisition();

    if (stop == 0 || 1)
    {

      if (acuser == 2 || local_mouv == 3)
        verif_mouv = 0;

      if (acuser_angle == 2 || local_mouv == 3)
        verif_angle = 0;

      if (acuser_recalage == 2 || local_mouv == 3)
      {
        verif_recalage = 0;
        etape = 0;
      }

      if (local_mouv != 7 && local_mouv != 8 && local_mouv != 11 && local_mouv != 12)
        etape = 0,verif_recalage=0;

      if (RPI == 0)
      {
        mouv = 0;
        local_mouv = 0;
      }

      if (liste_point[0].x == liste_point[1].x &&
          liste_point[0].y == liste_point[1].y)
      {
        if (local_mouv == 2)
        {
          mouv = 6;
          local_mouv = 6;
        }
        else if (local_mouv == 1)
        {
          mouv = 5;
          local_mouv = 5;
        }
      }

      switch (local_mouv)
      {
      case 0:
        PWM_G = moteur.gauche(0, Vg, 0, 0);
        PWM_D = moteur.droit(0, Vd, 0, 0);
        break;
      case 1:
        vlin = position.avant(dist, 0, teta_act, teta, 1);
        vang = position.angle(teta, teta_act, dist, 1);
        PWM_G = moteur.gauche(vlin + vang, Vg, 1, dist);
        PWM_D = moteur.droit(vlin - vang, Vd, 1, dist);
        break;
      case 2:
        vlin = position.arriere(dist, 0, teta_act, teta, 1);
        vang = position.angle(teta + PI, teta_act, dist, 1);
        PWM_G = moteur.gauche(vlin + vang, Vg, 1, dist);
        PWM_D = moteur.droit(vlin - vang, Vd, 1, dist);
        break;
      case 3:
        PWM_G = moteur.gauche(0, Vg, 0, 0);
        PWM_D = moteur.droit(0, Vd, 0, 0);
        break;
      case 4:
      {
        float err = limit_angle(teta_cons - teta_act);

        // Proche d'une bordure ?
        bool pres_bordure = (pos_act[0] < 200.0f || pos_act[0] > 2800.0f ||
                             pos_act[1] < 200.0f || pos_act[1] > 1800.0f);

        if (pres_bordure)
        {
          bool force_reverse = false;

          if (col_jaune == 1)
          {
            float angle_interdit = PI;
            float dist_to_forbidden;
            if (err > 0)
              dist_to_forbidden = limit_angle(angle_interdit - teta_act);
            else
              dist_to_forbidden = limit_angle(-angle_interdit - teta_act);

            if (err > 0 && dist_to_forbidden > 0 && dist_to_forbidden < err)
              force_reverse = true;
            if (err < 0 && dist_to_forbidden < 0 && dist_to_forbidden > err)
              force_reverse = true;
          }
          else if (col_bleu == 1)
          {
            float dist_to_forbidden = limit_angle(0.0f - teta_act);

            if (err > 0 && dist_to_forbidden > 0 && dist_to_forbidden < err)
              force_reverse = true;
            if (err < 0 && dist_to_forbidden < 0 && dist_to_forbidden > err)
              force_reverse = true;
          }

          if (force_reverse)
            err = -err;

          vang = constrain(err, -0.45f, 0.45f);
        }
        else
        {
          vang = position.angle(teta_cons, teta_act, 0, 2);
          vang = constrain(vang, -0.45f, 0.45f);
        }

        PWM_G = moteur.gauche(vang, Vg, 2, 0);
        PWM_D = moteur.droit(-vang, Vd, 2, 0);

        if (fabs(Vg) < 0.1f && fabs(Vd) < 0.1f &&
            fabs(limit_angle(teta_cons - teta_act)) < (PI / 40.0f))
          verif_angle = 1;

        break;
      }
      case 5:
        vlin = constrain(position.avant(dist, 0, teta_act, teta, 1), -0.4f, 0.4f);
        vang = constrain(position.angle(teta, teta_act, dist, 1), -0.3f, 0.3f);
        PWM_G = moteur.gauche(vlin + vang, Vg, 1, dist);
        PWM_D = moteur.droit(vlin - vang, Vd, 1, dist);
        break;
      case 6:
        vlin = constrain(position.avant(dist, 0, teta_act, teta, 1), -0.4f, 0.4f);
        vang = constrain(position.angle(teta + PI, teta_act, dist, 1), -0.3f, 0.3f);
        PWM_G = moteur.gauche(vlin + vang, Vg, 1, dist);
        PWM_D = moteur.droit(vlin - vang, Vd, 1, dist);
        break;
      case 7:
        Serial.printf("7 X%d Y%d", calX, calY);
        recalage(bleu);
        col_bleu = 1;
        break;
      case 8:
        Serial.printf("8 X%d Y%d", calX, calY);
        recalage(jaune);
        col_jaune = 1;
        break;
      case 9:
        curseur(bleu);
        break;
      case 10:
        curseur(jaune);
        break;
      case 11:
        if (!verif_recalage)
          recalage(X);
        break;
      case 12:
        if (!verif_recalage)
          recalage(Y);
        break;
      case 100:
        PWM_G = 0;
        PWM_D = 0;
        break;
      }
      old_mouv = local_mouv;
      // ── Anti-patinage ──────────────────────────────────────────────
      bool recalage_en_cours = (local_mouv == 7 || local_mouv == 8 ||
                                local_mouv == 11 || local_mouv == 12);

      if (!recalage_en_cours && !slip_active)
      {
        bool pwm_fort = (fabsf(PWM_D) > PWM_SLIP_THRESHOLD &&
                         fabsf(PWM_G) > PWM_SLIP_THRESHOLD);
        bool vitesse_nulle = (fabsf(Vd) < SPEED_SLIP_THRESHOLD &&
                              fabsf(Vg) < SPEED_SLIP_THRESHOLD);

        if (pwm_fort && vitesse_nulle)
        {
          if (slip_start_ms == 0)
            slip_start_ms = millis();
          else if (millis() - slip_start_ms > SLIP_DETECT_DELAY)
          {
            // Patinage confirmé → lancer la récupération
            slip_active = true;
            slip_origin_x = pos_act[0];
            slip_origin_y = pos_act[1];
            // Sens inverse du mouvement en cours
            slip_direction = (local_mouv == 2 || local_mouv == 6) ? 1 : -1;
            slip_start_ms = 0;
            Serial.println("[SLIP] Patinage détecté !");
          }
        }
        else
        {
          slip_start_ms = 0; // reset si conditions non réunies
        }
      }

      if (slip_active)
      {
        // Distance parcourue depuis le début du slip recovery
        float slip_dist = sqrtf(
            (pos_act[0] - slip_origin_x) * (pos_act[0] - slip_origin_x) +
            (pos_act[1] - slip_origin_y) * (pos_act[1] - slip_origin_y));

        if (slip_dist >= SLIP_DISTANCE)
        {
          slip_active = false;
          Serial.println("[SLIP] Récupération terminée.");
        }
        else
        {
          // Écrase les PWM calculés par le switch
          float v_recover = slip_direction * 0.35f;
          PWM_G = moteur.gauche(v_recover, Vg, 1, 0);
          PWM_D = moteur.droit(v_recover, Vd, 1, 0);
        }
      }
      // ── Fin anti-patinage ──────────────────────────────────────────
      if (etat_RPI == 1)
      {
        if (etape == 2)
        {
          md.setM1Speed(moteur.droit(-0.25f, Vg, 0, 0));
          md.setM2Speed(moteur.gauche(-0.25f, Vg, 0, 0));
        }
        else if (etape == 3)
        {
          md.setM1Speed(moteur.droit(0.25f, Vg, 0, 0));
          md.setM2Speed(moteur.gauche(0.25f, Vg, 0, 0));
        }
        else
        {
          if (mouv != 9 && mouv != 10 && mouv != 4)
          {
            PWM_D = limit_pwm(PWM_D, Vd);
            PWM_G = limit_pwm(PWM_G, Vg);
          }

          md.setM1Speed(PWM_D);
          md.setM2Speed(PWM_G);
        }
      }
      else
      {
        md.setM1Speed(0);
        md.setM2Speed(0);
      }
    }

    te = millis() - tempo;
    tempo = millis();

    Serial.printf("tic_d%dtic_g%d dist_d%f dist_g%f tour_d%f tour_g%f,teta_act%f X%.1f Y%.1f odo%.0f\n",
                  tic_droit, tic_gauche,
                  distance_droit, distance_gauche,
                  tour_droit, tour_gauche,
                  teta_act * 180.0f / PI,
                  pos_act[0], pos_act[1], err_odo);

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
  }
}

// ─────────────────────────────────────────────────────────────────────────
void recalage(int type)
{
  switch (type)
  {
  case X:
    teta_cons = (pos_act[0] > 1500) ? PI : 0;
    break;
  case Y:
    teta_cons = PI / 2.0f;
    break;
  case bleu:
    teta_cons = (calX == 0) ? PI : -PI / 2.0f;
    break;
  case jaune:
    teta_cons = (calX == 0) ? 0 : -PI / 2.0f;
    break;
  }

  if (type == X || type == Y)
  {
    switch (etape)
    {
    case 0:
      vang = position.angle(teta_cons, teta_act, 0, 2);
      PWM_G = moteur.gauche(vang, Vg, 0, 0);
      PWM_D = moteur.droit(-vang, Vd, 0, 0);
      if (fabs(Vg) < 0.05f && fabs(Vd) < 0.05f &&
          fabs(limit_angle(teta_cons - teta_act)) < (PI / 10.0f))
        etape = 2, temp = millis();
      break;
    case 2:
      if (diff_dist_droit == 0 && diff_dist_gauche == 0 &&
          (millis() - temp) > 1500)
      {
        if (type == X)
        {
          if (pos_act[0] > 1500)
          {
            teta_cons = PI;
            pos_act[0] = 2933;
          }
          else
          {
            teta_cons = 0;
            pos_act[0] = 67;
          }
        }
        else
        {
          teta_cons = PI / 2.0f;
          pos_act[1] = 67;
        }
        off_teta_act = teta_cons - teta_act;
        etape = 3;
      }
      break;
    case 3:
      switch (type)
      {
      case X:
        if (pos_act[0] > 1500)
        {
          if (pos_act[0] < 2800)
            verif_recalage = 1, etape = 0;
        }
        else
        {
          if (pos_act[0] > 150)
            verif_recalage = 1, etape = 0;
        }
        break;
      case Y:
        if (pos_act[1] > 150)
          verif_recalage = 1, etape = 0;
        break;
      }

      break;
    }
  }
  else
  {
    switch (etape)
    {
    case 0:
      vang = position.angle(teta_cons, teta_act, 0, 2);
      PWM_G = moteur.gauche(vang, Vg, 0, 0);
      PWM_D = moteur.droit(-vang, Vd, 0, 0);
      if (fabs(Vg) < 0.05f && fabs(Vd) < 0.05f &&
          fabs(limit_angle(teta_cons - teta_act)) < (PI / 10.0f))
        etape = 2, temp = millis(), moteur.reset();
      break;

    case 2:
      if (diff_dist_droit == 0 && diff_dist_gauche == 0 &&
          (millis() - temp) > 1500)
      {
        if (calX == 0)
        {
          if (type == bleu)
          {
            pos_act[0] = 2933;
            pos_act[1] = 1831;
          }
          else
          {
            pos_act[0] = 67;
            pos_act[1] = 1837;
          }

          // FIX RECALAGE : remise a zero complete
          diff_dist_droit = 0;
          diff_dist_gauche = 0;
          distance_droit = 0;
          distance_gauche = 0;
          tour_droit = 0;
          tour_gauche = 0;
          tic_droit = 0;
          tic_gauche = 0;
          old_d_co = 0;
          old_g_co = 0;
          diff_d_co = 0;
          diff_g_co = 0;
          old_dist_droit = 0;
          old_dist_gauche = 0;
          off_teta_act = 0;
          last_calc = millis();

          encoder2.setCount(0);
          encoder.setCount(0);

          teta_recu = (type == bleu) ? 180.0f : 0.0f;

          calX = 1;
          etape = 3;
        }
        else if (calY == 0)
        {
          diff_dist_droit = 0;
          diff_dist_gauche = 0;
          distance_droit = 0;
          distance_gauche = 0;
          tour_droit = 0;
          tour_gauche = 0;
          tic_droit = 0;
          tic_gauche = 0;
          old_d_co = 0;
          old_g_co = 0;
          diff_d_co = 0;
          diff_g_co = 0;
          old_dist_droit = 0;
          old_dist_gauche = 0;
          off_teta_act = 0;
          last_calc = millis();

          encoder2.setCount(0);
          encoder.setCount(0);
          teta_recu = -90.0f;
          pos_act[1] = 2000 - 67;
          calY = 1;

          etape = 3;
        }
      }
      break;

    case 3:
      if (fabs(distance_droit) > 100)
      {
        if (calY == 1)
        {
          etape = 4;
          verif_recalage = 1;
        }
        else
          etape = 0;
      }
      break;

    case 4:
      break;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────
void aquisition(void)
{
  tic_droit = encoder2.getCount();
  tic_gauche = encoder.getCount();

  tour_droit = (float)tic_droit / tic_tour_droit;
  tour_gauche = (float)tic_gauche / tic_tour_gauche;

  distance_droit = tour_droit * 48.0f * PI;
  distance_gauche = tour_gauche * 48.0f * PI;

  diff_d_co = distance_droit - old_d_co;
  diff_g_co = distance_gauche - old_g_co;

  old_d_co = distance_droit;
  old_g_co = distance_gauche;

  if (millis() - last_calc >= 10.0f)
  {
    diff_dist_droit = distance_droit - old_dist_droit;
    diff_dist_gauche = distance_gauche - old_dist_gauche;

    old_dist_droit = distance_droit;
    old_dist_gauche = distance_gauche;

    Vg = diff_dist_gauche / (millis() - last_calc);
    Vd = diff_dist_droit / (millis() - last_calc);
    last_calc = millis();
  }

  // FIX BUFFER : shift de droite a gauche, taille 6 → indices 0..4 surs
  for (int i = 4; i >= 0; i--)
  {
    moy_vitD[i + 1] = moy_vitD[i];
    moy_vitG[i + 1] = moy_vitG[i];
  }
  moy_vitD[0] = Vd;
  moy_vitG[0] = Vg;

  Vd = (moy_vitD[0] + moy_vitD[1] + moy_vitD[2]) / 3.0f;
  Vg = (moy_vitG[0] + moy_vitG[1] + moy_vitG[2]) / 3.0f;
  vitese = (Vd + Vg) / 2.0f;

  delta_x = liste_point[0].x - pos_act[0];
  delta_y = liste_point[0].y - pos_act[1];
  delta_x2 = liste_point[1].x - pos_act[0];
  delta_y2 = liste_point[1].y - pos_act[1];

  teta = atan2(delta_y, delta_x);
  teta2 = atan2(delta_y2, delta_x2);

  if (RPI == 1)
    teta_act = (teta_recu * PI / 180.0f) + ((-distance_droit + distance_gauche) / (20.0f * Rayon)) + off_teta_act;
  else
    teta_act = ((-distance_droit + distance_gauche) / (20.0f * Rayon)) - PI / 2.0f + off_teta_act;

  dist = sqrt((pos_fin[0] - pos_act[0]) * (pos_fin[0] - pos_act[0]) +
              (pos_fin[1] - pos_act[1]) * (pos_fin[1] - pos_act[1]));

  float delta_pos = (diff_d_co + diff_g_co) / 2.0f;
  pos_act[0] += cos(teta_act) * delta_pos;
  pos_act[1] += sin(teta_act) * delta_pos;
}

void updateEncoders()
{
  // ===== DROIT =====
  int64_t raw_droit = encoder2.getCount();
  int delta_droit = raw_droit - lastRaw_droit;
  lastRaw_droit = raw_droit;

  tic_droit += delta_droit;

  if (delta_droit > 0)
  {
    accumPos_droit += delta_droit;

    while (accumPos_droit >= 1000)
    {
      tic_droit -= 1;
      accumPos_droit -= 1000;
    }
  }
  else if (delta_droit < 0)
  {
    accumNeg_droit += -delta_droit;

    while (accumNeg_droit >= 1000)
    {
      tic_droit -= 1;
      accumNeg_droit -= 1000;
    }
  }

  // ===== GAUCHE =====
  int64_t raw_gauche = encoder.getCount();
  int delta_gauche = raw_gauche - lastRaw_gauche;
  lastRaw_gauche = raw_gauche;

  tic_gauche += delta_gauche;

  if (delta_gauche > 0)
  {
    accumPos_gauche += delta_gauche;

    while (accumPos_gauche >= 1000)
    {
      tic_gauche -= 1;
      accumPos_gauche -= 1000;
    }
  }
  else if (delta_gauche < 0)
  {
    accumNeg_gauche += -delta_gauche;

    while (accumNeg_gauche >= 1000)
    {
      tic_gauche -= 1;
      accumNeg_gauche -= 1000;
    }
  }
}

void curseur(int type)

{
  if (type == jaune)
  {

    switch (curseur_etape)
    {
    case 0:
      temp_curseur = millis();
      curseur_etape = 6;
      break;
    case 1:
      dist = sqrt((205 - pos_act[0]) * (205 - pos_act[0]) +
                  (285 - pos_act[1]) * (285 - pos_act[1]));
      teta = atan2(285 - pos_act[1], 205 - pos_act[0]);
      vlin = constrain(position.avant(dist, 0, teta_act, teta, 2), -0.4f, 0.4f);
      vang = constrain(position.angle(teta, teta_act, dist, 1), -0.25f, 0.25f);
      PWM_G = moteur.gauche(vlin - vang, Vg, 1, dist);
      PWM_D = moteur.droit(vlin + vang, Vd, 1, dist);
      if (Vg == 0 && Vd == 0 && millis() - temp_curseur > 800) // coordonées
      {
        // moteur.reset();
        curseur_etape = 2;
        temp_curseur = millis();
      }
      break;
    case 2:
      vang = position.angle(-35 * PI / 180.0f, teta_act, 0, Vg);
      PWM_G = moteur.gauche(-vang, Vg, 0, 0);
      PWM_D = moteur.droit(vang, Vd, 0, 0);
      if (fabs(Vg) < 0.05f && fabs(Vd) < 0.05f &&
          fabs(limit_angle(-35 * PI / 180.0f - teta_act)) < (PI / 40.0f))

      {
        // moteur.reset();
        curseur_etape = 3;
        temp_curseur = millis();
        pos_act[1] -= 15;
        // pos_act[0] -= 3;
        odo = millis();
      }
      break;
    case 3:
      PWM_G = 100;
      PWM_D = 170;
      if ((pos_act[0] > 420) || pos_act[1] < 150) // coordonées
      {
        curseur_etape = 4;
        PWM_G = moteur.gauche(0, Vg, 0, 0);
        PWM_D = moteur.droit(0, Vd, 0, 0);
        temp_curseur = millis();
        verif_curseur = 2;
      }
      break;
    case 4:
      PWM_G = moteur.gauche(0, Vg, 0, 0);
      PWM_D = moteur.droit(0, Vd, 0, 0);
      /*dist = sqrt((2700 - pos_act[0]) * (2700 - pos_act[0]) +
                   (180 - pos_act[1]) * (180 - pos_act[1]));
       teta = atan2(180 - pos_act[1], 2700 - pos_act[0]);
       vlin = constrain(position.avant(dist, 0, teta_act, teta,2), -0.4f, 0.4f);
       vang = constrain(position.angle(teta, teta_act, dist, 1), -0.25f, 0.25f);
       PWM_G = moteur.gauche(vlin - vang, Vg, 1);
       PWM_D = moteur.droit(vlin + vang, Vd, 1);*/
      if (Vg == 0 && Vd == 0 && millis() - temp_curseur > 1100) // coordonées
      {
        // moteur.reset();
        curseur_etape = 5;
        verif_curseur = 1;
        temp_curseur = millis();
        odo = millis();
        pos_act[0] += 7;
        // err_odo = 2;
      }
      break;
    case 5:
      PWM_G = moteur.gauche(0, Vg, 0, 0);
      PWM_D = moteur.droit(0, Vd, 0, 0);
      break;
    case 6:
      if (millis() - temp_curseur > 500) // coordonées
      {
        curseur_etape = 1;
        temp_curseur = millis();
      }
      break;
    }
  }
  else if (type == bleu)
  {
    switch (curseur_etape)
    {
    case 0:
      temp_curseur = millis();
      curseur_etape = 6;
      break;
    case 1:
      dist = sqrt((2795 - pos_act[0]) * (2795 - pos_act[0]) +
                  (285 - pos_act[1]) * (285 - pos_act[1]));
      teta = atan2(285 - pos_act[1], 2795 - pos_act[0]);
      vlin = constrain(position.avant(dist, 0, teta_act, teta, 2), -0.4f, 0.4f);
      vang = constrain(position.angle(teta, teta_act, dist, 1), -0.25f, 0.25f);
      PWM_G = moteur.gauche(vlin - vang, Vg, 1, dist);
      PWM_D = moteur.droit(vlin + vang, Vd, 1, dist);
      if (Vg == 0 && Vd == 0 && millis() - temp_curseur > 800) // coordonées
      {
        // moteur.reset();
        curseur_etape = 2;
        temp_curseur = millis();
      }
      break;
    case 2:
      vang = position.angle(-145 * PI / 180.0f, teta_act, 0, Vg);
      PWM_G = moteur.gauche(-vang, Vg, 0, 0);
      PWM_D = moteur.droit(vang, Vd, 0, 0);
      if (fabs(Vg) < 0.05f && fabs(Vd) < 0.05f &&
          fabs(limit_angle(-145 * PI / 180.0f - teta_act)) < (PI / 40.0f))

      {
        // moteur.reset();
        curseur_etape = 3;
        temp_curseur = millis();
        pos_act[1] -= 15;
        pos_act[0] -= 3;
        odo = millis();
      }
      break;
    case 3:
      PWM_G = 170;
      PWM_D = 100;
      if ((pos_act[0] < 2520) || pos_act[1] < 175) // coordonées
      {
        curseur_etape = 4;
        PWM_G = moteur.gauche(0, Vg, 0, 0);
        PWM_D = moteur.droit(0, Vd, 0, 0);
        temp_curseur = millis();
        verif_curseur = 2;
      }
      break;
    case 4:
      PWM_G = moteur.gauche(0, Vg, 0, 0);
      PWM_D = moteur.droit(0, Vd, 0, 0);
      /*dist = sqrt((2700 - pos_act[0]) * (2700 - pos_act[0]) +
                   (180 - pos_act[1]) * (180 - pos_act[1]));
       teta = atan2(180 - pos_act[1], 2700 - pos_act[0]);
       vlin = constrain(position.avant(dist, 0, teta_act, teta,2), -0.4f, 0.4f);
       vang = constrain(position.angle(teta, teta_act, dist, 1), -0.25f, 0.25f);
       PWM_G = moteur.gauche(vlin - vang, Vg, 1);
       PWM_D = moteur.droit(vlin + vang, Vd, 1);*/
      if (Vg == 0 && Vd == 0 && millis() - temp_curseur > 1100) // coordonées
      {
        // moteur.reset();
        curseur_etape = 5;
        verif_curseur = 1;
        temp_curseur = millis();
        odo = millis();
        err_odo = 2;
      }
      break;
    case 5:
      PWM_G = moteur.gauche(0, Vg, 0, 0);
      PWM_D = moteur.droit(0, Vd, 0, 0);
      break;
    case 6:
      if (millis() - temp_curseur > 500) // coordonées
      {
        curseur_etape = 1;
        temp_curseur = millis();
      }
      break;
    }
  }
}

float limit_pwm(float pwm, float speed)
{
  const float SPEED_LOW = 0.01f;
  const float SPEED_HIGH = 0.4f;
  const float PWM_STOP = 110.0f; /* limite sous 0.01          */
  const float PWM_JUMP = 110.0f; /* palier dès 0.01 dépassé   */
  const float PWM_MAX = 350.0f;

  /* Pas de limitation si signes opposés (décélération/freinage)
     ou si la vitesse est nulle */
  if (speed * pwm <= 0.0f)
    return pwm;

  float abs_speed = fabsf(speed);
  float limit;

  if (abs_speed < SPEED_LOW)
  {
    limit = PWM_STOP;
  }
  else if (abs_speed < SPEED_HIGH)
  {
    /* Interpolation linéaire : 140 → 300 sur [0.1, 0.4] */
    float t = (abs_speed - SPEED_LOW) / (SPEED_HIGH - SPEED_LOW);
    limit = PWM_JUMP + t * (PWM_MAX - PWM_JUMP);
  }
  else
  {
    limit = PWM_MAX;
  }

  /* Clamp symétrique */
  if (pwm > limit)
    return limit;
  if (pwm < -limit)
    return -limit;
  return pwm;
}

void reception(char ch)
{
  if (ch == '0')
    vcons = 0.0f;
  if (ch == '1')
    vcons = 0.1f;
  if (ch == '2')
    vcons = 0.2f;
  if (ch == '3')
    vcons = 0.3f;
  if (ch == '4')
    vcons = 0.4f;
  if (ch == '5')
    vcons = 0.5f;
  if (ch == '6')
    vcons = -0.1f;
  if (ch == '7')
    vcons = -0.2f;
  if (ch == '8')
    vcons = -0.3f;
  if (ch == '9')
    vcons = -0.4f;
  if (ch == 'r')
    moteur.reset();
}