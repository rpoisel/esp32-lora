// Erweitert für KHS (KFLECT Handsteuerung)
// WZ @ Nov.2018

/* Example from Sandeep Mistry 
 * With mods from AliExpress/LilyGo docs
 * For TTGo ESP32 LoRa-OLED board
 * http://www.lilygo.cn/down_view.aspx?TypeId=11&Id=78&Fid=t14:11:14
 * Based on SX1276 LoRa Radio
 * http://www.semtech.com/apps/product.php?pn=SX1276
 * RMB 29Nov2017
 */

#include <SPI.h>
#include <LoRa.h>       // https://github.com/sandeepmistry/arduino-LoRa
#include <U8g2lib.h>   // https://github.com/olikraus/U8g2_Arduino
#include <HardwareSerial.h>

#define OFF 0   // For LED
#define ON 1

// SPI LoRa Radio
#define LORA_SCK 5        // GPIO5 - SX1276 SCK
#define LORA_MISO 19     // GPIO19 - SX1276 MISO
#define LORA_MOSI 27    // GPIO27 -  SX1276 MOSI
#define LORA_CS 18     // GPIO18 -   SX1276 CS
#define LORA_RST 14   // GPIO14 -    SX1276 RST
#define LORA_IRQ 26  // GPIO26 -     SX1276 IRQ (interrupt request)

// I2C OLED Display works with SSD1306 driver
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16

/* Pick One. Hardware I2C does NOT work! This article helped: https://robotzero.one/heltec-wifi-kit-32/ 
* Some OLED displays don't handle ACK correctly so SW I2C works better. Thank you Olikraus!
* TTGo OLED has pin 16 reset unlike other OLED displays
*/

// UNCOMMENT one of the constructor lines below
//U8X8_SSD1306_128X64_NONAME_SW_I2C Display(/* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST); // Unbuffered, basic graphics, software I2C
//U8G2_SSD1306_128X64_NONAME_1_SW_I2C Display(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST); // Page buffer, SW I2C
U8G2_SSD1306_128X64_NONAME_F_SW_I2C Display(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST); // Full framebuffer, SW I2C

const int blueLED = LED_BUILTIN;
const int Button = 0;

String SerNum = "00";  // derzeit noch fix codiert, später einstellbar
String FWESP32LoRa = "FWES_0.4";

String rssi = "";
String packet = "";

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
bool isCommand = false;
bool isError = false;

//Definition der Schnittstelle zum Nano
HardwareSerial SerialNano(1);


// =================================================================================
void setup() {
  pinMode(blueLED, OUTPUT); // For LED feedback
  pinMode(Button, INPUT);

  Serial.begin(115200);
  while (!Serial);
  Serial.println();
  Serial.println("KFLECT 3D-Spiegel Handsteuerung");
  Serial.println("#"+SerNum+FWESP32LoRa);

  SerialNano.begin(115200,SERIAL_8N1,37,25);  // TX=25 nicht verw. Nano
  inputString.reserve(50);

  // Very important for SPI pin configuration!
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS); 
  
  // Very important for LoRa Radio pin configuration! 
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);         

  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  
  // The larger the spreading factor the greater the range but slower data rate
  // Send and receive radios need to be set the same
  LoRa.setSpreadingFactor(7); // ranges from 6-12, default 7 see API docs

  // ev. Bandbreite hier einstellen
  // ...
  
  // Change the transmit power of the radio
  // Default is LoRa.setTxPower(17, PA_OUTPUT_PA_BOOST_PIN);
  // Most modules have the PA output pin connected to PA_BOOST, gain 2-17
  // TTGO and some modules are connected to RFO_HF, gain 0-14
  // If your receiver RSSI is very weak and little affected by a better antenna, change this!
  LoRa.setTxPower(14, PA_OUTPUT_RFO_PIN);

      // send packet
      LoRa.beginPacket();
      LoRa.print("#"+SerNum+FWESP32LoRa);
      LoRa.endPacket();

  Display.begin();
  Display.enableUTF8Print();    // enable UTF8 support for the Arduino print() function
  Display.setFont(u8g2_font_unifont_t_latin);
  // Display Info
  Display.clearBuffer();  
  Display.setCursor(0,12); Display.print("Handsteuerung");
  Display.setCursor(0,30); Display.print(FWESP32LoRa);
  Display.setCursor(0,48); Display.print("standby");
  Display.sendBuffer();

}

// =================================================================================
void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {    // received a packet
    packet = "";                   // Clear packet
    while (LoRa.available()) {      // read packet
      packet += (char)LoRa.read(); // Assemble new packet
    }
    rssi = LoRa.packetRssi();
    Serial.print("Received packet:");
    Serial.println(packet);
    Serial.println("@RSSI=" + rssi);

    if (packet.substring(5,7)=="ER") {
      isError = true;
      digitalWrite(blueLED, ON);  // Turn blue LED on
    } else { 
      isError = false;
      digitalWrite(blueLED, ON);
    }
  }
  
  if (SerialNano.available()) {
    // get the new byte:
    char inChar = (char)SerialNano.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }

    
  if (stringComplete) {
    if (inputString[0]=='#') {  // nur Befehle anzeigen und weiterleiten
      Serial.print("COMMAND:");
      Serial.print(inputString);
      // Display Info
      Display.clearBuffer();  
      Display.setCursor(0,11); Display.print("Handsteuerung");
      Display.setCursor(0,24); Display.print(FWESP32LoRa);
      Display.setCursor(0,36); Display.print((String)inputString);
      // if (inputString=='#') // ev. Standby erkennen
      Display.sendBuffer();
      
      // send packet
      LoRa.beginPacket();
      LoRa.print((String)inputString);
      LoRa.endPacket();
    }
    inputString = ""; // clear the string
    stringComplete = false;
  }

  if (!isError) {
    digitalWrite(blueLED, !digitalRead(Button));  // monitor Button on blueLED
  }

}

// =================================================================================


