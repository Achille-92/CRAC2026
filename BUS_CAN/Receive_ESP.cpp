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
    if (CAN.packetExtended()) {
      Serial.print("Extended packet with id 0x");
    } else {
      Serial.print("Standard packet with id 0x");
    }

    Serial.print(CAN.packetId(), HEX);
    Serial.print(" and length ");
    Serial.println(packetSize);

    Serial.print("Data: ");
    while (CAN.available()) {
      Serial.print((char)CAN.read());
    }
    Serial.println();
  }
}
