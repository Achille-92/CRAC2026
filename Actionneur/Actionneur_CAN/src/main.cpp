#include <Arduino.h>
#include <CAN.h>

#define TX_GPIO_NUM   5
#define RX_GPIO_NUM   4

void reception(char ch);

int num_carte = 0;
int etat_RPI = 0, etat_ESP_RPI = 0, on_pour_rpi = 0;
int action_a_faire = 0, sous_pince = 0, verif_action = 0, ack_action = 0;

void setup() {
  Serial.begin(115200);
  Serial.print("Bonjour");

  CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);

  // start the CAN bus at 500 kbps
  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
  else {
    Serial.println("CAN a demarré");
  }
}

void loop() {

  if (Serial.available()>0)
  {
    reception(Serial.read());
  }


  // Lecture CAN centralisée
  int packetSize = CAN.parsePacket();
  if (packetSize){
    uint32_t canId = CAN.packetId();

    if (canId == 0x01) {
      CAN.readBytes((uint8_t*)&etat_RPI, packetSize);
      //Serial.printf("\nRPI : %d",etat_RPI);
    }
    
    if (canId == 0x500+num_carte) {
      CAN.readBytes((uint8_t*)&action_a_faire, packetSize);
    }

    if (canId == 0x502+num_carte) {
      CAN.readBytes((uint8_t*)&sous_pince, packetSize);
    }

    if (canId == 0x504+num_carte) {
      CAN.readBytes((uint8_t*)&ack_action, packetSize);
    }
  }

  switch (etat_ESP_RPI){
    case 0: 
      on_pour_rpi = 1; // Variable pour dire à la RPI que la carte Actionneur fonctionne
      if (etat_RPI == 1) {
        Serial.println("RPI lancée");
        etat_ESP_RPI = 1;
      } else {
        Serial.println("Attente de la RPI");
        verif_action = 0;
      }
      break;
    
    case 1:
      if (etat_RPI == 0 || etat_RPI == 2) {
        etat_ESP_RPI = 0;
        Serial.println("Retour à l'état 0 (RPI arrêtée)");
      }
      Serial.printf("\nverif_action : %d",verif_action);
      if(ack_action == 2){
        verif_action = 0;
      }
      break;
    }

    /*Serial.printf("\nAction : %d",action_a_faire);
    Serial.printf("\nSous_pince %d\n",sous_pince);*/

    static unsigned long lastSend = 0;
    if (millis() - lastSend > 50) {  // toutes les 100 ms
      lastSend = millis();

      CAN.beginPacket(0x003);
      CAN.write((uint8_t*)&on_pour_rpi, sizeof(int));   // 4 octets
      CAN.endPacket();

      CAN.beginPacket(0x109);
      CAN.write((uint8_t*)&verif_action, sizeof(int));   // 4 octets
      CAN.endPacket();

    }

}

void reception(char ch)
{

  static int i = 0;
  static String chaine = "";
  String commande;
  int index, length;

  if ((ch == 13) or (ch == 10))
  {
    index = chaine.indexOf(' ');
    length = chaine.length();
    if (index == -1)
    {
      commande = chaine;
    }
    else
    {
      commande = chaine.substring(0, index);
    }

    if (commande == "a")
    {
      verif_action = 1;
    }
    
    chaine = "";
  }
  else
  {
    chaine += ch;
  }
}