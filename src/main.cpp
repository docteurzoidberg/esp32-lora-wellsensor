#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <math.h>
#include <axp20x.h>

#include <dzb/CRC.h>
#include <dzb/Packet.h>
#include <dzb/PacketType.h>
#include <dzb/PacketWriter.h>
#include <dzb/PacketReader.h>
#include <dzb/Serialization.h>

#define SERIAL_SPEED 115200

#define SCK     5
#define MISO    19
#define MOSI    27
#define SS      18
#define RST     14
#define DI00    26

#define SDA_PIN 21
#define SCL_PIN 22

#define ECHO_PIN 32
#define TRIG_PIN 33

#define BAND    433E6
#define PABOOST true

float distance = 0.0f;
float batVoltage = 0.0f;
bool axpFound = false;

void sendPacket(dzb::Packet const& packet);

dzb::PacketWriter writer(dzb::id_t{ 'P', 'T' }, 48 /* bytes, threshold */, sendPacket);

AXP20X_Class axp;

void sendPacket(dzb::Packet const& packet) {
  LoRa.beginPacket();
  LoRa.write(packet.buffer.data(), packet.buffer.size());
  LoRa.endPacket(true); // async
  //packetCounter++;
}

void readValues() {

  // You can use isBatteryConnect() to check whether the battery is connected properly
  if (axp.isBatteryConnect()) {

      batVoltage = axp.getBattVoltage();

      Serial.println("CONNECT");

      // Get battery voltage
      Serial.print("BAT Voltage:");
      Serial.print(batVoltage);
      Serial.println(" mV");

      // To display the charging status, you must first discharge the battery,
      // and it is impossible to read the full charge when it is fully charged
      if (axp.isChargeing()) {
        Serial.print("Charge:");
        Serial.print(axp.getBattChargeCurrent());
        Serial.println(" mA");
      } else {
        // Show current consumption
        Serial.print("Discharge:");
        Serial.print(axp.getBattDischargeCurrent());
        Serial.println(" mA");
        /*getBattPercentage just only support axp202 */
        //if (slave_address == AXP202_SLAVE_ADDRESS) {
        //    Serial.print("Per: ");
        //    Serial.print(axp.getBattPercentage());
        //    Serial.println(" %");
        //}
      }
  } else {
      Serial.println("DISCONNECT");
  }


  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long pulseDuration = pulseIn(ECHO_PIN, HIGH);
  distance = pulseDuration /58;
}

void loraReport() {

  Serial.print("[LORA SND]");

  //send distance if > 0
  if(distance>0) {
    writer.write(dzb::PacketType::DISTANCE, uint16_t { (uint16_t) distance });
  }

  //send battery level if axp present
  if(axpFound) {
    writer.write(dzb::PacketType::BATT_VOLTAGE, float{ batVoltage });
  }

  //send packet
  writer.flush();
}

void setup() {

  dzb::init_packet_type_meta();
  dzb::init_crc_table();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);

  LoRa.setPins(SS,RST,DI00);
  Serial.begin(SERIAL_SPEED);
  SPI.begin(SCK,MISO,MOSI,SS);
  Wire.begin(SDA_PIN, SCL_PIN);

  axpFound = !axp.begin(Wire, AXP192_SLAVE_ADDRESS);
  if (axpFound) {

      // ! DC1 is the power supply of ESP32, do not control it
      // axp.setDCDC1Voltage(3300);  //esp32 core VDD    3v3
      // axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);

      //Setting LDO2 and LOD3 3300mV
      axp.setLDO2Voltage(3300);   //LORA VDD     3v3
      axp.setLDO3Voltage(3300);   //GPS VDD      3v3

      //Enable LDO2 , It controls the lora power
      axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
      //Enable LDO3, It controls the GPS power
      axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);

      //Idle the power supply, turn it off
      axp.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);
      axp.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);
  }
  else {
    Serial.println("AXP192 not found !");
  }

  if (!LoRa.begin(BAND)){
    Serial.println("Starting LoRa failed!");
    Serial.flush();
    delay(1000);
    ESP.restart();
  }
}

void loop() {
  readValues();
  loraReport();
  delay(10000);
}