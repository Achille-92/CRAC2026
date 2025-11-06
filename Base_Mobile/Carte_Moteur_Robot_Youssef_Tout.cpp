#include <Arduino.h>
#include <Wire.h>
#include <DualVNH5019MotorShield.h>
#include <ESP32Encoder.h>
#include <BluetoothSerial.h>
#include <CAN.h>

#define TX_GPIO_NUM 5
#define RX_GPIO_NUM 4
#define LIMITE_VIT 170

int signe(float a);
int ensemble(float valeur, float delta);
int comparer(float valeur_comparer, float valeur_compareur, float delta);
void Asserv(void *);
// void calcul_traj(void *);

struct coo
{
  float x;
  float y;
};
coo traj[1000] = {0, 0};
coo liste_point[100] = {0, 0};
int tic_gauche = 0;
int tic_droit = 0;
float tour_gauche = 0;
float tour_droit = 0;
float distance_droit = 0, old_dist_droit = 0, diff_dist_droit = 0;
float distance_gauche = 0, old_dist_gauche = 0, diff_dist_gauche = 0;
float dist = 0, teta = 0, delta_x = 0, delta_y = 0, teta_act = 0, diff_teta = 0, teta_act_deg = 0, teta_deg = 0, diff_deg = 0;
float pos_act[2] = {105, 120}, pos_fin[2] = {200005, 120}, pos_traj[2], E = 0, dist_a_parcourir = 10, off_teta = 0, off_teta_act = 0;
int VIT_gauche = 0, VIT_droit = 0, i = 0, S_VIT_d = 0, S_VIT_g = 0, old_VIT_droit = 0, old_VIT_gauche = 0;
float kpd = 3.0, kdd = 200.0, kpg = 3.0, kdg = 200.0, kpda = 4, kpga = 4, kdga = 20.0, kdda = 20.0, old_diff_dista = 0;
float diff_dist_d = 0, diff_dist_g = 0, diff_dista = 0, old_diff_dist_d = 0, old_diff_dist_g = 0, VIT_Ad = 0, VIT_Ag = 0, teta_cons = 0;
int cons = 0, dist_cons = 200, arriver = 0, suivi = 1;
/*const float tic_tour_droit = 4121.7, tic_tour_gauche = 4052, Rayon = 11;*/
int packetSize = 0, fin = 100, n = 0;
float Vg = 0, Vd = 0, diff_Vg = 0, diff_Vd = 0, old_diff_Vg = 0, old_diff_Vd = 0, VIT = 0;

const float tic_tour_droit = 3164.0, tic_tour_gauche = 3167.7, Rayon = 11.2;
float teta_recu = -181, teta_cons_recu = -181;
int etat_ESP_RPI = 0, etat_RPI = 0;
static bool x_ok = false, y_ok = false, angle_ok = false;
bool mouvement = false;
int nbr_point = 0;
bool trajectoire_recue = false;

// BluetoothSerial Bt;
DualVNH5019MotorShield md;
ESP32Encoder encoder;
ESP32Encoder encoder2;
TaskHandle_t Asservissement;
// TaskHandle_t trajectoire;

void setup()
{
  Serial.begin(115200);
  // Bt.begin("TEST");

  md.init(); // Initialise le shield
  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  // use pin 19 and 18 for the first encoder
  encoder.attachFullQuad(36, 39);
  // use pin 17 and 16 for the second encoder
  encoder2.attachFullQuad(23, 22);

  // set starting count value after attaching
  encoder.setCount(0);
  encoder2.setCount(0);
  md.setSpeeds(0, 0);

  // clear the encoder's raw count and set the tracked count to zero
  //
  Serial.println("Encoder Start = " + String((int32_t)encoder.getCount()));
  xTaskCreate(Asserv, "Asserv", 32000, NULL, 15, &Asservissement);
  // xTaskCreate(calcul_traj, "traj", 32000, NULL, 10, &trajectoire);
  CAN.setPins(RX_GPIO_NUM, TX_GPIO_NUM);

  // start the CAN bus at 500 kbps
  if (!CAN.begin(1000E3))
  {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }
  liste_point[0] = {300, 400};
  liste_point[1] = {1000, 400};
  liste_point[2] = {1100, 1400};
  liste_point[3] = {2000, 1400};
}

void loop() {
  //liste_point[12].x = 145;
  // Lecture CAN centralisée
  int packetSize = CAN.parsePacket();
  if (packetSize) {
    uint32_t canId = CAN.packetId();

    if (canId == 0x01 && (packetSize == 4 || packetSize == 1)) {
      CAN.readBytes((uint8_t*)&etat_RPI, packetSize);
      while (CAN.available()) CAN.read(); // vider le buffer
    }

    if (canId == 0x200 && packetSize == 4) {
      CAN.readBytes((uint8_t*)&pos_act[0], 4);
      Serial.print("Reçu X : ");
      Serial.println(pos_act[0]);
    }

    if (canId == 0x201 && packetSize == 4) {
      CAN.readBytes((uint8_t*)&pos_act[1], 4);
      Serial.print("Reçu Y : ");
      Serial.println(pos_act[1]);
    }

    if (canId == 0x202 && packetSize == 4) {
      CAN.readBytes((uint8_t*)&teta_recu, 4);
      Serial.print("Reçu Angle : ");
      Serial.println(teta_recu, 2);
      teta_act = teta_recu * PI / 180.0;
    }

    if (canId == 0x203 && packetSize == 4) {
      CAN.readBytes((uint8_t*)&pos_fin[0], 4);
      Serial.print("Reçu Y : ");
      Serial.println(pos_act[1]);
    }
    if (canId == 0x204 && packetSize == 4) {
      CAN.readBytes((uint8_t*)&pos_fin[1], 4);
      Serial.print("Reçu Y : ");
      Serial.println(pos_act[1]);
    }
    if (canId == 0x205 && packetSize == 4) {
      CAN.readBytes((uint8_t*)&teta_cons_recu, 4);
      Serial.print("Reçu Angle_voulu : ");
      Serial.println(teta_cons_recu, 2);
      //teta_act = teta_recu * PI / 180.0;
    }

    if (canId == 0x207 && (packetSize == 4 || packetSize == 1)) {
      CAN.readBytes((uint8_t*)&nbr_point, packetSize);
      Serial.print("Reçu nbr_point : ");
      Serial.println(nbr_point, 2);
      trajectoire_recue = false;  // Réinitialiser le flag
    }

    // Réception des coordonnées X des points (IDs paires : 0x208, 0x20A, 0x20C, ...)
    if (canId >= 0x208 && canId <= 0x2FE && canId % 2 == 0 && packetSize == 4) {
      int index = (canId - 0x208) / 2;  // Calculer l'index du point
      if (index < 50) {  // Sécurité pour ne pas dépasser la taille du tableau
        CAN.readBytes((uint8_t*)&liste_point[index].x, 4);
        Serial.print("Reçu X[");
        Serial.print(index);
        Serial.print("] : ");
        Serial.println(liste_point[index].x);
      }
    }

    // Réception des coordonnées Y des points (IDs impaires : 0x209, 0x20B, 0x20D, ...)
    if (canId >= 0x209 && canId <= 0x2FF && canId % 2 == 1 && packetSize == 4) {
      int index = (canId - 0x209) / 2;  // Calculer l'index du point
      if (index < 50) {  // Sécurité
        CAN.readBytes((uint8_t*)&liste_point[index].y, 4);
        Serial.print("Reçu Y[");
        Serial.print(index);
        Serial.print("] : ");
        Serial.println(liste_point[index].y);
        
        // Vérifier si on a reçu tous les points
        if (index == nbr_point - 1) {
          trajectoire_recue = true;
          Serial.println("✅ Trajectoire complète reçue !");
          
        }
      }
    }
  }

  switch (etat_ESP_RPI){
    case 0:
      mouvement= false;
      if (pos_act[0] != -1) x_ok = true;
      if (pos_act[1] != -1) y_ok = true;
      if (teta_recu != -181) angle_ok = true;

      if (!x_ok || !y_ok || !angle_ok) {
        Serial.println("En attente des coordonnées initiales depuis la RPi...");
      }

      // Test en continu de la condition de passage à l'état 1
      if (x_ok && y_ok && angle_ok) {
        if (etat_RPI == 1) {
          Serial.println("Coordonnées reçues, passage à l'état 1 !");
          etat_ESP_RPI = 1;
          encoder.setCount(0);
          encoder2.setCount(0);
          distance_droit = distance_gauche = 0;
        } else {
          Serial.println("Coordonnées OK mais RPi pas encore prête (etat_RPI != 1)");
        }
      }
      break;
    
    
    case 1:
      mouvement = true;
      if (etat_RPI == 0 || etat_RPI == 2) {
        etat_ESP_RPI = 0;
        x_ok = false, y_ok = false, angle_ok = false;
        pos_act[0] = -1;
        pos_act[1] = -1;
        teta_recu = -181;
        Serial.println("Retour à l'état 0 (RPi arrêtée)");
      }

      Serial.printf("X=%.1f Y=%.1f Angle=%.1f X_voulu=%.1f Y_voulu=%1.f Angle_voulu=%.1f Nbr_point=%d\n",pos_act[0],pos_act[1],teta_act_deg,pos_fin[0],pos_fin[1],teta_cons_recu,nbr_point);

      float x = pos_act[0];
      float y = pos_act[1];
      float teta = teta_act_deg;
      static unsigned long lastSend = 0;
      if (millis() - lastSend > 100) {  // toutes les 100 ms
        lastSend = millis();

        CAN.beginPacket(0x100);
        CAN.write((uint8_t*)&x, sizeof(float));   // 4 octets
        CAN.endPacket();

        CAN.beginPacket(0x101);
        CAN.write((uint8_t*)&y, sizeof(float));   // 4 octets
        CAN.endPacket();

        CAN.beginPacket(0x102);
        CAN.write((uint8_t*)&teta, sizeof(float)); // 4 octets
        CAN.endPacket();
      }

      break;
    }
}

void Asserv(void *)
{
  int finish = 1;
  while (1)
  {

    tic_droit = encoder2.getCount();
    tic_gauche = encoder.getCount();
    tour_droit = (float)tic_droit / tic_tour_droit;
    tour_gauche = (float)tic_gauche / tic_tour_gauche;
    old_dist_droit = distance_droit;
    old_dist_gauche = distance_gauche;
    distance_droit = tour_droit * 40.0 * PI;
    distance_gauche = tour_gauche * 40.0 * PI;
    diff_dist_droit = distance_droit - old_dist_droit;
    diff_dist_gauche = distance_gauche - old_dist_gauche;
    Vg = diff_dist_gauche / 5.0;
    Vd = diff_dist_droit / 5.0;
    old_diff_Vd = diff_Vd;
    old_diff_Vg = diff_Vg;
    diff_Vg = VIT - Vg;
    diff_Vd = VIT - Vd;

    delta_x = pos_fin[0] - pos_act[0];
    delta_y = pos_fin[1] - pos_act[1];
    teta = atan2(delta_y, delta_x) + off_teta;
    teta_act = ((distance_droit - distance_gauche) / (20 * Rayon)) + off_teta_act;
    teta_act_deg = (teta_act * 180.0) / PI;
    teta_deg = (teta * 180.0) / PI;
    diff_deg = (diff_teta * 180.0) / PI;
    ////////////////////////////////////////angle modulo 2PI
    if (teta_act > PI)
      teta_act -= 2.0 * PI;
    if (teta_act < -PI)
      teta_act += 2.0 * PI;

    if (teta_act_deg > 180)
      teta_act_deg -= 360;
    if (teta_act_deg < -180)
      teta_act_deg += 360;

    if (teta > PI)
      teta -= 2.0 * PI;
    else if (teta < -PI)
      teta += 2.0 * PI;
    
    if (teta_deg > 180)
      teta_deg -= 360;
    if (teta_deg < -180)
      teta_deg += 360;

    dist = sqrt(((pos_fin[0] - pos_act[0]) * (pos_fin[0] - pos_act[0])) + ((pos_fin[1] - pos_act[1]) * (pos_fin[1] - pos_act[1])));
    old_diff_dist_d = diff_dist_d;
    old_diff_dist_g = diff_dist_g;
    old_diff_dista = diff_dista;
    diff_dista = diff_teta * 2 * PI * Rayon;
    pos_act[0] = pos_act[0] + (cos(teta_act) * ((diff_dist_droit + diff_dist_gauche) / 2));
    pos_act[1] = pos_act[1] + (sin(teta_act) * ((diff_dist_gauche + diff_dist_droit) / 2));
    old_VIT_droit = VIT_droit;
    old_VIT_gauche = VIT_gauche;
    /*
        pos_fin[0] = traj[suivi].x;
        pos_fin[1] = traj[suivi].y;
        if (comparer(pos_act[0], pos_fin[0], 10) && comparer(pos_act[1], pos_fin[1], 10))
        {
          suivi += 1;
        }*/
    switch (cons)
    {
    case 0:
      diff_teta = teta - teta_act;
      diff_dista = diff_teta * 2 * PI * Rayon;
      diff_dist_d = dist - diff_dist_droit;
      diff_dist_g = dist - diff_dist_gauche;
      if (ensemble((diff_teta), (PI / 1.0)))
      {

        if (comparer(pos_act[0], pos_fin[0], 100) && comparer(pos_act[1], pos_fin[1], 100))
        {
          VIT_droit = (diff_teta) * (kpd * diff_dist_d + kdd * (diff_dist_d - old_diff_dist_d));
          VIT_gauche = (diff_teta) * (kpg * diff_dist_g + kdg * (diff_dist_g - old_diff_dist_g));
          VIT_Ad = 0;
          VIT_Ag = 0;
        }
        else
        {
          VIT_droit = pow(cos(diff_teta), 3) * (kpd * diff_dist_d + kdd * (diff_dist_d - old_diff_dist_d));
          VIT_gauche = pow(cos(diff_teta), 3) * (kpg * diff_dist_g + kdg * (diff_dist_g - old_diff_dist_g));
          VIT_Ad = -1*(1 * kpda * (diff_dista) + kdda * (diff_dista - old_diff_dista));
          VIT_Ag = kpga * (diff_dista)+kdga * (diff_dista - old_diff_dista);
        }
      }
      else
      {
        VIT_Ad = -2 * (1 * kpda * (diff_dista) + kdda * (diff_dista - old_diff_dista));
        VIT_Ag = 2 * kpga * (diff_dista)+2 * kdga * (diff_dista - old_diff_dista);
        VIT_droit = 0;
        VIT_gauche = 0;
      }

      break;
    case 1:
      diff_teta = teta - teta_act + PI;
      diff_dista = diff_teta * 2 * PI * Rayon;
      diff_dist_d = dist - diff_dist_droit;
      diff_dist_g = dist - diff_dist_gauche;
      if (ensemble((diff_teta), (PI / 1.0)))
      {

        if (comparer(pos_act[0], pos_fin[0], 100) && comparer(pos_act[1], pos_fin[1], 100))
        {
          VIT_droit = cos(diff_teta + PI) * (kpd * diff_dist_d + kdd * (diff_dist_d - old_diff_dist_d));
          VIT_gauche = cos(diff_teta + PI) * (kpg * diff_dist_g + kdg * (diff_dist_g - old_diff_dist_g));
          VIT_Ad = 0;
          VIT_Ag = 0;
        }
        else
        {
          VIT_droit = pow(cos(diff_teta + PI), 3) * (kpd * diff_dist_d + kdd * (diff_dist_d - old_diff_dist_d));
          VIT_gauche = pow(cos(diff_teta + PI), 3) * (kpg * diff_dist_g + kdg * (diff_dist_g - old_diff_dist_g));
          VIT_Ad = -1*(1 * kpda * (diff_dista) + kdda * (diff_dista - old_diff_dista));
          VIT_Ag = kpga * (diff_dista)+kdga * (diff_dista - old_diff_dista);
        }
      }
      else
      {
        VIT_Ad = -2 * (1 * kpda * (diff_dista) + kdda * (diff_dista - old_diff_dista));
        VIT_Ag = 2 * kpga * (diff_dista)+2 * kdga * (diff_dista - old_diff_dista);
        VIT_droit = 0;
        VIT_gauche = 0;
      }

      break;
    case 2:
      diff_teta = teta_cons - teta_act;
      diff_dista = diff_teta * 2 * PI * Rayon;
      VIT_Ad = -2 * (1 * kpda * (diff_dista) + kdda * (diff_dista - old_diff_dista));
      VIT_Ag = 2 * kpga * (diff_dista)+2 * kdga * (diff_dista - old_diff_dista);
      VIT_droit = 0;
      VIT_gauche = 0;
      break;
    case 3:
      diff_teta = teta - teta_act;
      diff_dist_d = dist - diff_dist_droit;
      diff_dist_g = dist - diff_dist_gauche;
      VIT_droit = cos(diff_teta) * (kpd * diff_dist_d + kdd * (diff_dist_d - old_diff_dist_d));
      VIT_gauche = cos(diff_teta) * (kpg * diff_dist_g + kdg * (diff_dist_g - old_diff_dist_g));
      VIT_Ad = 0;
      VIT_Ag = 0;
      break;
    default:
      cons = 0;
      break;
    }
    if (VIT_droit - old_VIT_droit > 5)
      VIT_droit = old_VIT_droit + signe(VIT_droit) * 5;
    if (VIT_gauche - old_VIT_gauche > 5)
      VIT_gauche = old_VIT_gauche + signe(VIT_gauche) * 5;
    if (ensemble(diff_teta, PI / 180.0) && !ensemble(diff_teta, (10 * PI) / 180.0))
      VIT_Ad += signe(VIT_Ad) * 40, VIT_Ag += signe(VIT_Ag) * 40;
    if (VIT_droit > abs(cos(diff_teta)) * LIMITE_VIT)
      VIT_droit = abs(cos(diff_teta)) * LIMITE_VIT;
    if (VIT_gauche > abs(cos(diff_teta)) * LIMITE_VIT)
      VIT_gauche = abs(cos(diff_teta)) * LIMITE_VIT;
    if (VIT_droit < -abs(cos(diff_teta)) * LIMITE_VIT)
      VIT_droit = -abs(cos(diff_teta)) * LIMITE_VIT;
    if (VIT_gauche < -abs(cos(diff_teta)) * LIMITE_VIT)
      VIT_gauche = -abs(cos(diff_teta)) * LIMITE_VIT;

    if (VIT_Ad > LIMITE_VIT)
      VIT_Ad = LIMITE_VIT;
    if (VIT_Ag > LIMITE_VIT)
      VIT_Ag = LIMITE_VIT;
    if (VIT_Ad < -LIMITE_VIT)
      VIT_Ad = -LIMITE_VIT;
    if (VIT_Ag < -LIMITE_VIT)
      VIT_Ag = -LIMITE_VIT;
    S_VIT_g = VIT_gauche + VIT_Ag;
    S_VIT_d = VIT_droit + VIT_Ad;

    if (S_VIT_d < 20 && S_VIT_d > -20)
      S_VIT_d = 0;
    else
      S_VIT_d += signe(S_VIT_d) * 20;
    if (S_VIT_g < 20 && S_VIT_g > -20)
      S_VIT_g = 0;
    else
      S_VIT_g += signe(S_VIT_g) * 20;
    if (cons == -1)
      VIT_Ad = 0, VIT_Ag = 0, VIT_droit = 0, VIT_gauche = 0;
      
    if (mouvement == true){
      md.setM2Speed((S_VIT_d) / 2.0);

      md.setM1Speed((S_VIT_g) / 2.0);

      if (VIT_Ad == 0 && VIT_Ag == 0 && VIT_droit == 0 && VIT_gauche == 0)
        md.setBrakes(400, 400);
    }
    else {
      md.setM2Speed(0);

      md.setM1Speed(0);

    }
    //Serial.printf("PWM_G:%d PWM_D:%d X:%.1f Y%.1f diff_teta%.1f,cos:%.1f diff%.1f\n", S_VIT_g, S_VIT_d, pos_act[0], pos_act[1], diff_deg, cos(diff_teta),diff_dist_d);
    vTaskDelay(pdMS_TO_TICKS(5));
  }
} /*
 void calcul_traj(void *)
 {
   int num_point = 0, fin2 = 0;
   float dist1 = 0, dist2 = 0, teta1 = 0, teta2 = 0;
   dist1 = sqrt(((liste_point[0].x - pos_act[0]) * (liste_point[0].x - pos_act[0])) + ((liste_point[0].y - pos_act[1]) * (liste_point[0].y - pos_act[1])));
   dist2 = sqrt(((liste_point[1].x - liste_point[0].x) * (liste_point[1].x - liste_point[0].x)) + ((liste_point[1].y - liste_point[0].y) * (liste_point[1].y - liste_point[0].x)));

   teta1 = atan2((liste_point[0].y-pos_act[1]), (liste_point[0].x-pos_act[0]));
   teta2 = atan2((liste_point[1].y-liste_point[0].y), (liste_point[1].x-liste_point[0].x));
   if (((pos_traj[0] != liste_point[0].x) || (pos_traj[1] != liste_point[0].y)))
   {
     fin = dist1 / 50.0;
     dist_a_parcourir = dist1 / fin;
     n = 0;
     traj[0].x = pos_act[0];
     traj[0].y = pos_act[1];
     while ((comparer(traj[fin].x, liste_point[0].x, 2) == 0) || (comparer(traj[fin].y, liste_point[0].y, 2) == 0))
     {
       // Serial.printf("X=%f Y=%f dist=%f dista=%f n=%d\n", traj[n].x, traj[n].y, dist, dist_a_parcourir, n);
       traj[n + 1].x = traj[n].x + cos(teta1) * dist_a_parcourir;
       traj[n + 1].y = traj[n].y + sin(teta1) * dist_a_parcourir;
       n++;
     }
     pos_traj[0] = traj[n].x;
     pos_traj[1] = traj[n].y;
     fin2 = fin + dist2 / 50.0;
     dist_a_parcourir = dist2 / fin2;
     n = fin;
     while ((comparer(traj[fin2].x, liste_point[1].x, 2) == 0) || (comparer(traj[fin2].y, liste_point[1].y, 2) == 0))
     {
       // Serial.printf("X=%f Y=%f dist=%f dista=%f n=%d\n", traj[n].x, traj[n].y, dist, dist_a_parcourir, n);
       traj[n + 1].x = traj[n].x + cos(teta2) * dist_a_parcourir;
       traj[n + 1].y = traj[n].y + sin(teta2) * dist_a_parcourir;
       n++;
     }
     suivi = 10;
     // Serial.printf("X=%f Y=%f dist=%f dista=%f n=%d\n", traj[n].x, traj[n].y, dist, dist_a_parcourir, n);
   }

   vTaskDelay(pdMS_TO_TICKS(20));
 }*/
int signe(float a)
{
  if (a >= 0)
    return 1;
  else
    return -1;
}
int comparer(float valeur_comparer, float valeur_compareur, float delta)
{
  if ((valeur_comparer) > (valeur_compareur - delta) && (valeur_comparer) < (valeur_compareur + delta))
    return 1;
  else
    return 0;
}

int ensemble(float valeur, float delta)
{
  if ((valeur) > (-delta) && (valeur) < (delta))
    return 1;
  else
    return 0;
}
