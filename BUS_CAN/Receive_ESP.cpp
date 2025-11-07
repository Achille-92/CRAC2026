#include <CAN.h>

#define RX_GPIO_NUM  4
#define TX_GPIO_NUM  5

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(1000);

  Serial.println("CAN Receiver");

  CAN.setPins(RX_GPIO_NUM, TX_GPIO_NUM);

  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  Serial.println("CAN ready to receive...");
}

void loop() {

  int packetSize = CAN.parsePacket();
  if (packetSize) {
    uint32_t canId = CAN.packetId();

    if (canId == 0x200 && packetSize == 4) {
      CAN.readBytes((uint8_t*)&pos_act[0], 4);
      Serial.print("Reçu X : ");
      Serial.println(pos_act[0]);
    }
  }
}
