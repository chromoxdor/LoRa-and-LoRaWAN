//************* OPTIONS ************************************************************************
#define NODE_NR 1  //give your node a number

#define LORA 1
#define LORA_GATEWAY 0
#define LORA_FREQUENCY 868E6  //frequency in Hz (433E6, 868E6, 915E6)

#define LORAWAN 0
#define JOIN_RETRY 100  //how often should the node try to join a gatway until it will go \
                        // to sleep indefinitely

#define SCALE 1
#define CALIBWEIGHT 37  // weight used for calibration the scale in gramm

#define SHT 0  // use an SHT2X I2C device

#define DS18B20_TEMP 0   // set to 1 if one or multiple sensors are connected to one pin
#define DS18B20_TEMP2 0  // set to 1 if one or two sensors (on different pins) are connected

#define SLEEP_TIME 0.2;  // sleep time in minutes
#define ACK_BEEP 1       // if an acknowledge beep is needed when data was sent
#define SERIAL 1         // saves some memory if not needed
#define LONGPRESS_S 4    // seconds to longpressevent
//**********************************************************************************************


#if LORA_GATEWAY
#undef LORAWAN
#undef LORA
#endif

#if LORA
#undef LORAWAN
#endif

#if LORA || LORAWAN
#include "SPI.h"
char buffer[10];
#endif

#if LORA || LORA_GATEWAY
#include "LoRa.h"        //https://github.com/sandeepmistry/arduino-LoRa
const int csPin = 10;    // LoRa radio chip select
const int resetPin = 9;  // LoRa radio reset
const int irqPin = 3;    // change for your board; must be a hardware interrupt pin
#endif

#if LORAWAN
#include <lorawan.h>  //https://github.com/ElectronicCats/Beelan-LoRaWAN
// OTAA credentials
const char *devEui = "0000000000000000";
const char *appEui = "0000000000000000";
const char *appKey = "00000000000000000000000000000000";
char outStr[255];
byte recvStatus = 0;

const sRFM_pins RFM_pins = {
  .CS = 10,
  .RST = 9,
  .DIO0 = 3,
  .DIO1 = 4,
  .DIO2 = 5,
  .DIO5 = -1,
};

bool isJoined;
int joinCount = 0;
#endif

#if SCALE
#include "HX711.h"  //https://github.com/RobTillaart/HX711
HX711 myScale;
uint8_t dataPin = A2;
uint8_t clockPin = A1;
float weight;
float scale;
float tare;
uint32_t offset;
unsigned long currentCalibTime = 0;
#endif

#if SHT
#include "Wire.h"
#include "tinySHT2x.h"  //https://github.com/RobTillaart/tinySHT2x
tinySHT2x sht;
float tempSHT;
float humSHT;
#endif

#if DS18B20_TEMP
#include <OneWire.h>
#include "DS18B20.h"  //https://github.com/matmunk/DS18B20
DS18B20 ds(A3);
float tempC1;
float tempC2;
#endif

#if DS18B20_TEMP2
#include <microDS18B20.h>  //https://github.com/GyverLibs/microDS18B20
MicroDS18B20<A3> sensor1;
MicroDS18B20<8> sensor2;
float tempC1;
float tempC2;
#endif

#include <EEPROM.h>
#include "LowPower.h"  //https://github.com/rocketscream/Low-Power

int counter = 0;
int Pcounter = 0;

const int BUTTON_PIN = 2;
const int BUZZER_PIN = 6;
const int MOSFET = 7;

bool sensReady = false;
bool woke = true;
bool clicked = false;
bool calib = false;
bool longPressed = false;
bool manuelWake = false;

unsigned long sensorTime;

float voltage;
float sleepTime = SLEEP_TIME;  // sleep time in minutes

#include "OneButton.h"  //https://github.com/mathertel/OneButton
OneButton button(BUTTON_PIN, true);


//***********************************************************************************************
//                                                                                        Setup
//***********************************************************************************************

void setup() {
  pinMode(MOSFET, OUTPUT);
  digitalWrite(MOSFET, HIGH);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

#if !LORAWAN
  ack(100);
#endif

#if SERIAL || LORA_GATEWAY
  Serial.begin(9600);
#endif
  ////------------------------------------------------------------lora init

#if LORA || LORA_GATEWAY
  LoRa.setPins(csPin, resetPin, irqPin);
#if SERIAL && LORA
  Serial.println("Starting LoRa Node");
#endif
#if SERIAL && LORA_GATEWAY
  Serial.println("Starting LoRa Gateway");
#endif

  if (!LoRa.begin(LORA_FREQUENCY)) {
#if SERIAL
    Serial.println("Starting LoRa failed!");
#endif
    while (1)
      ;
  }
#if LORA
  LoRa.onTxDone(onTxDone);
#endif
#if LORA_GATEWAY
  // register the receive callback
  LoRa.onReceive(onReceive);
  // put the radio into receive mode
  LoRa.receive();
#endif
#endif


#if LORAWAN
  if (!lora.init()) {
#if SERIAL
    Serial.println("RFM95 not detected");
#endif
    delay(5000);
    return;
  }
  // Set LoRaWAN Class change CLASS_A or CLASS_C
  lora.setDeviceClass(CLASS_A);
  // Set Data Rate
  lora.setDataRate(SF9BW125);
  // set channel to random
  lora.setChannel(MULTI);
  // Put OTAA Key and DevAddress here
  lora.setDevEUI(devEui);
  lora.setAppEUI(appEui);
  lora.setAppKey(appKey);
  tryJoining();
#endif

////------------------------------------------------------------scale init
#if SCALE && !LORA_GATEWAY
  myScale.begin(dataPin, clockPin);
  EEPROM.get(1, offset);
  EEPROM.get(6, scale);
  EEPROM.get(12, tare);
  if (isnan(tare)) {
    tare = 0;
  }
  myScale.set_offset(offset);
  myScale.set_scale(scale);
#if SERIAL
  Serial.print("\noffset(");
  Serial.print(offset);
  Serial.print("); scale(");
  Serial.print(scale, 6);
  Serial.println(");\n");
  Serial.print("); tare(");
  Serial.print(tare);
  Serial.println(");\n");
  delay(100);
#endif
#endif

//------------------------------------------------------------buttons init
#if SCALE
  button.attachLongPressStart(LongPressStart);
#endif
  button.attachClick(click);
  button.setPressMs(LONGPRESS_S * 1000);

  //------------------------------------------------------------sht init
#if SHT
  Wire.begin();
  sht.begin();
#endif

  //------------------------------------------------------------initial read and send
  gatherData();
}


//***********************************************************************************************
//                                                                                  Join LoRaWan
//***********************************************************************************************

#if LORAWAN
void tryJoining() {
  ack(200);
  joinCount = 0;
  bool isJoined;
  do {
#if SERIAL
    Serial.println("Joining...");
    Serial.println(joinCount);
    delay(100);
#endif
    isJoined = lora.join();
    joinCount++;
    ack(60);
    if (joinCount > 1) {
      delay(8000);
    }
    if (isJoined) { joinCount = 1000; }
  } while (joinCount < JOIN_RETRY);

  if (isJoined) {
#if SERIAL
    Serial.println("Joined to network");
#endif
    ack(1000);
  } else {
#if SERIAL
    Serial.println("Sleeping forever...");
#endif
    sleepForever();
  }
}
#endif


//***********************************************************************************************
//                                                                                       SLEEP
//***********************************************************************************************

#if !LORA_GATEWAY
void sleep()  // here we put the arduino to sleep
{
#if SERIAL
  delay(200);
#endif
  powerDown();
  manuelWake = false;
  woke = false;
  digitalWrite(MOSFET, LOW);
  // Allow wake up pin to trigger interrupt on low.
  attachInterrupt(0, wakeUp, LOW);
  // Enter power down state with ADC and BOD module disabled.
  // Wake up when wake up pin is low.
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  // Disable external pin interrupt on wake up pin.
  detachInterrupt(0);
}

#if LORAWAN
void sleepForever()  // here we put the arduino to sleep
{
  powerDown();
  attachInterrupt(0, wakeUp, LOW);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  detachInterrupt(0);
  tryJoining();
}
#endif

void wakeUp()  // here the interrupt is handled after wakeup
{
  manuelWake = true;
  //execute code here after wake-up before returning to the loop() function
  // timers and code using timers (serial.print and more...) will not work here.
  //digitalWrite(LED, LOW);
}

//------------------------------------------------------------sleep counter
void sleepCounter() {
#if SERIAL
  Serial.println(counter);
  delay(100);
#endif
  if (counter < ceil(sleepTime * 7.5)) {  //(minutes * 60 / 8) since the mcu can only go to sleep for 8sec via watchdog timer
    woke = false;
    counter++;
    sleep();
  } else {
    counter = 0;
    woke = true;
    gatherData();
  }
}

//***********************************************************************************************
//                                                                                      POWER UP
//***********************************************************************************************
void powerUp() {

  button.tick();
#if SERIAL
  Serial.println("Powering up");
#endif

#if LORAWAN
  lora.wakeUp();
#endif

#if SCALE
  myScale.power_up();
#endif

  digitalWrite(MOSFET, HIGH);
}


//***********************************************************************************************
//                                                                                    POWER DOWN
//***********************************************************************************************

void powerDown() {

#if LORA
  LoRa.sleep();
#endif

#if LORAWAN
  lora.sleep();
#endif

#if SCALE
  myScale.power_down();
#endif
}


//***********************************************************************************************
//                                                                                         RESET
//***********************************************************************************************

//used for rejoining the lorawan
void (*resetFunc)(void) = 0;  //declare reset function @ address 0
#endif                        //#if !LORA_GATEWAY


//***********************************************************************************************
//                                                                                          LOOP
//***********************************************************************************************

void loop() {

#if !LORA_GATEWAY

  button.tick();
  if (!manuelWake && !woke) {  //wake via watchdog
    sleepCounter();
  }
  //------------------------------------------------------------go and get data
  if (clicked && !calib) {
    ack(100);
    clicked = false;
    gatherData();
  }

  //------------------------------------------------------------calibrate scale
#if SCALE
  if (calib && longPressed) {
    calibrate();
  }
#endif

#endif  //#if !LORA_GATEWAY
}


//***********************************************************************************************
//                                                                          Gather and send data
//***********************************************************************************************

#if !LORA_GATEWAY
void gatherData() {
  powerUp();
  //------------------------------------------------------------battery
  int sensorValue = analogRead(A0);
  //voltage = sensorValue * (3.91 / 377.0); //v1 board value
  voltage = sensorValue * (4.2 / 900.0);
#if SERIAL
  Serial.print("sensorValue:");
  Serial.println(sensorValue);
  Serial.print("Voltage:");
  Serial.println(voltage);
#endif

  //------------------------------------------------------------ds18b20
#if DS18B20_TEMP
  int cTemp = 1;
  tempC1 = -127;
  tempC2 = -127;
  while (ds.selectNext()) {
    if (cTemp = 1) {
      tempC1 = ds.getTempC();
    } else {
      tempC2 = ds.getTempC();
    }
    cTemp++;
  }
#if SERIAL
  Serial.print("temp 1:");
  Serial.println(tempC1);
  Serial.print("temp 2:");
  Serial.println(tempC2);
#endif
#endif

#if DS18B20_TEMP2
  sensor1.requestTemp();
  sensor2.requestTemp();
  tempC1 = -127;
  tempC2 = -127;
  if (sensor1.readTemp()) tempC1 = sensor1.getTemp();
  if (sensor2.readTemp()) tempC2 = sensor2.getTemp();
#if SERIAL
  Serial.print("temp 1:");
  Serial.println(tempC1);
  Serial.print("temp 2:");
  Serial.println(tempC2);
#endif
#endif

  //------------------------------------------------------------scale
#if SCALE
  sensorTime = millis();
#if SERIAL
  Serial.println("...waiting for scale");
#endif
  do {
    sensReady = myScale.is_ready();
    delay(1);
#if SERIAL
    Serial.print(".");
#endif
  } while (!sensReady || millis() - sensorTime < 300);

  if (sensReady) {
    weight = myScale.get_units(1) - tare;
#if SERIAL
    Serial.println();
    Serial.print("absWeight: ");
    Serial.println(myScale.get_units(1));
    Serial.print("tareWeight: ");
    Serial.println(weight);
#endif
  } else {
#if SERIAL
    Serial.print("scale was not ready");
#endif
  }
#endif

  //------------------------------------------------------------SHT
#if SHT
  sensorTime = millis();
#if SERIAL
  Serial.println("...waiting for SHT");
#endif
  /*do {
      sensReady = sensor.measure();
      delay(1);
      Serial.print(".");
    } while (!sensReady || millis() - sensorTime < 300) if (sensReady) {*/
  tempSHT = sht.getTemperature(65);
  humSHT = sht.getHumidity(28);
#if SERIAL
  Serial.print("Temperature (°C): ");
  Serial.println(tempSHT);
  Serial.print("Humidity (%RH): ");
  Serial.println(humSHT);
#endif
#endif

  //------------------------------------------------------------send data
#if LORA
#if SERIAL
  Serial.print("Sending packet (LoRa)...");
  Serial.println(Pcounter);
#endif
  String payload = makePayload();
#if SERIAL
  Serial.print("Sending: ");
  Serial.println(payload);
#endif
  // send in async / non-blocking mode
  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket(true);  // true = async / non-blocking mode
#endif

#if LORAWAN
#if SERIAL
  Serial.print("Sending packet (LoRaWan)...");
#endif
  String payload = makePayload();
  const char *payloadCStr = payload.c_str();
  size_t payloadLength = strlen(payloadCStr);

#if SERIAL
  Serial.print("Sending: ");
  Serial.println(payloadCStr);
#endif
  //sending
  lora.sendUplink(payloadCStr, payloadLength, 0, 1);

#if SERIAL
  Serial.println("TxDone");
#endif
#endif

#if !LORA
#if SERIAL
  Serial.print("...going to sleep");
#endif
#if ACK_BEEP
  ack(10);
#endif
  sleep();
#else
#endif
}

#endif  //#if !LORA_GATEWAY


//***********************************************************************************************
//                                                                                  Make payload
//***********************************************************************************************

#if LORA || LORAWAN
String makePayload() {
  String myStr;
  myStr = String(NODE_NR) + ",";
  dtostrf(voltage, 3, 2, buffer);
  myStr += String(buffer) + ",";
#if DS18B20_TEMP || DS18B20_TEMP2
  dtostrf(tempC1, 3, 1, buffer);
  myStr += String(buffer) + ",";
  dtostrf(tempC2, 3, 1, buffer);
  myStr += String(buffer) + ",";
#endif
#if SHT
  dtostrf(tempSHT, 3, 1, buffer);
  myStr += String(buffer) + ",";
  myStr += String((int)humSHT) + ",";
#endif
#if SCALE
  dtostrf(weight, 3, 1, buffer);
  myStr += String(buffer) + ",";
#endif
  myStr.remove(myStr.length() - 1);
  return myStr;
}
#endif


//***********************************************************************************************
//                                                                                  Sent package
//***********************************************************************************************
#if LORA
void onTxDone() {

#if ACK_BEEP
  ack(10);
#endif
#if SERIAL
  Serial.println("TxDone");
  Serial.println("...going to sleep");
#endif
  sleep();
}
#endif


//***********************************************************************************************
//                                                                                         Click
//***********************************************************************************************

void click() {
#if SERIAL
  Serial.println("click");
#endif
  clicked = true;
#if LORAWAN
#if SERIAL
  Serial.println("resetting");
#endif
  resetFunc();
#endif
}


//***********************************************************************************************
//                                                                                    Long Click
//***********************************************************************************************

#if SCALE

void LongPressStart() {
#if SERIAL
  Serial.println("longPress");
#endif
  longPressed = true;
  calib = true;
  //if (!calib){calibrate();}
}

#endif

//***********************************************************************************************
//                                                                                       Beeping
//***********************************************************************************************

void blinker(int a) {
  for (int i = 0; i < (a * 2); i++) {
    digitalWrite(BUZZER_PIN, i % 2);
    if (i < a * 2) {
      delay(200);
    }
  }
}


//***********************************************************************************************
//                                                                                      Ack beep
//***********************************************************************************************

void ack(int t) {
#if SERIAL
  Serial.print("beep: ");
  Serial.println(t);
#endif

  digitalWrite(BUZZER_PIN, LOW);
  delay(t);
  digitalWrite(BUZZER_PIN, HIGH);
}


//***********************************************************************************************
//                                                                                  LoRA Gateway
//***********************************************************************************************

#if LORA_GATEWAY
void onReceive(int packetSize) {
  // read packet
  for (int i = 0; i < packetSize; i++) {
    Serial.print((char)LoRa.read());
  }

  // print RSSI of packet
  Serial.print(",");
  Serial.print(LoRa.packetRssi());
  Serial.print(",");
  Serial.println(LoRa.packetSnr());
}
#endif


//***********************************************************************************************
//                                                                               Calibrate scale
//***********************************************************************************************

#if SCALE
void calibrate() {
  powerUp();
  longPressed = false;
  calib = true;
  blinker(2);
  weight = myScale.get_units(1);
#if SERIAL
  Serial.println("Tare scale...");
#endif
  myScale.tare(20);  // average 20 measurements.

  offset = myScale.get_offset();
  ack(100);

#if SERIAL
  Serial.print("Press the button for ");
  Serial.print(LONGPRESS_S);
  Serial.println(" seconds to start the calibration");
#endif
  currentCalibTime = millis();
  while (!longPressed) {
    button.tick();
    if (millis() - currentCalibTime > 15000) {
      calib = false;
      break;
    }
  }
  longPressed = false;

  if (calib) {
    blinker(4);
#if SERIAL
    Serial.print("Place exactly ");
    Serial.print(CALIBWEIGHT);
    Serial.print(" on the scale and press the button for ");
    Serial.print(LONGPRESS_S);
    Serial.println(" seconds");
#endif
    currentCalibTime = millis();
    while (!longPressed) {
      button.tick();
      if (millis() - currentCalibTime > 300000) {
        calib = false;
        break;
      }
    }
    if (calib) {
      blinker(1);
      //uint32_t weightCalib = 5000;
      int weightCalib = CALIBWEIGHT;

      //Serial.print("WEIGHT: ");
      //Serial.println(weightCalib);
      myScale.calibrate_scale(weightCalib, 20);
      scale = myScale.get_scale();
#if SERIAL
      Serial.print("SCALE:  ");
      Serial.println(scale, 6);
#endif
      EEPROM.put(1, offset);
      EEPROM.put(6, scale);
      myScale.set_offset(offset);
      myScale.set_scale(scale);

#if SERIAL
      Serial.print("offset(");
      Serial.print(offset);
      Serial.print("); and scale(");
      Serial.print(scale, 6);
      Serial.println(")");
#endif

      //Serial.println("\n\n");
      blinker(6);
      calib = false;
    }
  } else {
#if SERIAL
    Serial.println("Only set new tare");
#endif
    EEPROM.put(12, weight);
  }
  delay(100);
  longPressed = false;
  sleep();
}

#endif