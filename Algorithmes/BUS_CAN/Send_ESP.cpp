#include <CAN.h>

#define TX_GPIO_NUM   5
#define RX_GPIO_NUM   4

void setup() {
  Serial.begin (115200);
  while (!Serial);
  delay (1000);

  Serial.println ("CAN Sender");

  // Set the pins
  CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);

  // start the CAN bus at 500 kbps
  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
}

void loop() {
  // send packet: id is 11 bits, packet can contain up to 8 bytes of data
  Serial.print("Sending packet ... ");

  CAN.beginPacket(0x102);
  CAN.write((uint8_t*)&teta, sizeof(float)); // 4 octets
  CAN.endPacket();

  Serial.println("done");

  delay(1000);

}
