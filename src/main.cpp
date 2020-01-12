#include <HardwareSerial.h>
#include <SPI.h>
#include <LoRa.h> // https://github.com/sandeepmistry/arduino-LoRa

// SPI LoRa Radio
#define LORA_SCK 5   // GPIO5 - SX1276 SCK
#define LORA_MISO 19 // GPIO19 - SX1276 MISO
#define LORA_MOSI 27 // GPIO27 -  SX1276 MOSI
#define LORA_CS 18   // GPIO18 -   SX1276 CS
#define LORA_RST 14  // GPIO14 -    SX1276 RST
#define LORA_IRQ 26  // GPIO26 -     SX1276 IRQ (interrupt request)

HardwareSerial SerialNano(1);

// =================================================================================
void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("HELLO");

  SerialNano.begin(115200, SERIAL_8N1, 37, 25); // TX=25 nicht verw. Nano

  // Very important for SPI pin configuration!
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  // Very important for LoRa Radio pin configuration!
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  if (!LoRa.begin(868E6, true))
  {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }

  LoRa.setSpreadingFactor(7); // ranges from 6-12, default 7 see API docs
  LoRa.setTxPower(14, PA_OUTPUT_RFO_PIN);
}

// =================================================================================
void loop()
{
  static unsigned long lastAction = millis();
  static auto cnt = 0;
  if (millis() - lastAction > 1000)
  {
    String msg("Hello ");
    msg += cnt;
    Serial.println(msg);
    LoRa.beginPacket();
    LoRa.print(msg);
    LoRa.endPacket();
    lastAction = millis();
    cnt++;
  }
}

// =================================================================================
