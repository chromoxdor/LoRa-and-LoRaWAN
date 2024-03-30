//**********************************************************************************************
//************* OPTIONS ************************************************************************

//common-setting-section------------------------------------------------------------------------
#define LORA_FREQUENCY 868E6  //frequency in Hz (433E6, 868E6, 915E6)
#define NODE_NR "1"           //give your node a number or a name


//LoRa-section----------------------------------------------------------------------------------
#define LORA 1
#define TX_RETRIES 2  // How often should the LoRa node try to send when no ack is received until \
                      // the system goes to sleep again (interval for retry is 8 seconds)
                      // It will then return to it´s given "sleep-time" interval. 
#define LORA_GATEWAY 0

#define ENCRYPTION 0           //add simple AES128 encryption to LoRa
#define PASSWORD "MyPassword"  //enter a password and make sure it matches the one of \
                               // the gateway/node
#define COMMON_PHRASE "CXD"    // Gateway checks if received message contains this phrase \
                               // at the end and only  reacts to that

//LoRaWAN-section-------------------------------------------------------------------------------
#define LORAWAN 0
#define DEVEUI "0000000000000000"
#define APPEUI "0000000000000000"
#define APPKEY "00000000000000000000000000000000"
#define JOIN_RETRY 100  //how often should the node try to join a gateway until it will go \
                        // to sleep indefinitely

//sensor-section--------------------------------------------------------------------------------
#define SCALE 1
#define CALIBWEIGHT 10000  // weight used for calibrating the scale in gramm

#define SHT 1  // use an SHT2X I2C device

#define DS18B20_TEMP 0   // set to 1 if one or multiple sensors are connected to one pin
#define DS18B20_TEMP2 1  // set to 1 if one or two sensors (on different pins) are connected


//extras-section--------------------------------------------------------------------------------
#define SLEEP_TIME 0          // sleep time in minutes (0 will cause a 8 sec intervall)
#define ACK_BEEP 1            // if an acknowledge beep is needed when data was sent
#define SERIAL 0              // saves some memory if not needed since it is only for debugging
#define LONGPRESS_S 5         // seconds to longpressevent
#define MAX_PAYLOAD_SIZE 128  // Maximum size of the payload buffer
#define SERIAL_SPEED 9600     // baud rate (e.g. 9600, 19200, 38400, 57600, 115200)
#define PARITY SERIAL_8N1     // set serial mode (e.g. SERIAL_8N1, SERIAL_8E1 ,SERIAL_8O1)
//**********************************************************************************************
//**********************************************************************************************


#if LORA_GATEWAY
#undef LORAWAN
#undef LORA
#undef SCALE
#undef SHT
#undef DS18B20_TEMP 0
#undef DS18B20_TEMP2 1
#endif

#if LORA
bool isReceived = true;
bool triedOnce = false;
int nrOfTries = 0;
//#undef LORAWAN
#endif

#if LORA || LORAWAN
#include "SPI.h"
char buffer[10];
unsigned long rxTime;
unsigned long nrOfMsgs;
String checkStr;
String payload;
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
const char* devEui = DEVEUI;
const char* appEui = APPEUI;
const char* appKey = APPKEY;

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
bool waitForAck = false;
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

#if ENCRYPTION
#include <AESLib.h>
#include "Base64.h"  // Include the Base64 library

void convertPassphraseToKey(const char* passphrase, uint8_t* key) {
  int passphraseLength = strlen(passphrase);
  if (passphraseLength < 16) {
    // If passphrase is shorter than 16 bytes, repeat it to fill the key
    for (int i = 0; i < 16; i++) {
      key[i] = passphrase[i % passphraseLength];
    }
  } else if (passphraseLength == 16) {
    // If passphrase is exactly 16 bytes, copy it directly to the key
    memcpy(key, passphrase, 16);
  } else {
// If passphrase is longer than 16 bytes, truncate it and issue a warning
#warning "Passphrase is longer than 16 bytes. Truncating to 16 bytes."
    memcpy(key, passphrase, 16);
  }
}
uint8_t key[16];
#endif

#if (PARITY == O)
#define SERIAL_MODE "SERIAL_8O1"
#elif (PARITY == E)
#define SERIAL_MODE "SERIAL_8E1"
#else
#define SERIAL_MODE "SERIAL_8N1"
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
unsigned long upTime;

float voltage;
float sleepTime = SLEEP_TIME;  // sleep time in minutes

#include "OneButton.h"  //https://github.com/mathertel/OneButton
OneButton button(BUTTON_PIN, true);

//------------------------------------------------------------encrytion init
#if ENCRYPTION

#endif


//***********************************************************************************************
//                                                                                        Setup
//***********************************************************************************************

void setup() {
  pinMode(MOSFET, OUTPUT);
  digitalWrite(MOSFET, HIGH);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

#if !LORAWAN
  ack(100);
#endif

  Serial.begin(SERIAL_SPEED, PARITY);
  // Check if a device is connected
  if (!Serial) {
    // If no device is connected, stop serial communication
    Serial.end();
  }

  //------------------------------------------------------------encrytion key
#if ENCRYPTION
  convertPassphraseToKey(PASSWORD, key);
  Serial.print("Password: ");
  for (int i = 0; i < 16; ++i) {
    Serial.print((char)key[i]);
  }
  Serial.println();
  Serial.flush();
#endif

  //------------------------------------------------------------lora init
#if LORA || LORA_GATEWAY
  LoRa.setPins(csPin, resetPin, irqPin);
#if LORA
  Serial.println("Starting LoRa Node");
#endif
#if LORA_GATEWAY
  Serial.println("Starting LoRa Gateway");
#endif

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
#if LORA || LORA_GATEWAY
  LoRa.onTxDone(onTxDone);
  LoRa.onReceive(onReceive);
#endif
#if LORA_GATEWAY
  // put the radio into receive mode
  LoRa.receive();
#endif
#endif


#if LORAWAN
  if (!lora.init()) {
    Serial.println("RFM95 not detected");
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
  delayIfSerial();
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
#if !LORA_GATEWAY
  gatherData();
#endif
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
    delayIfSerial();
#endif
    if (joinCount >= 1) {
      delay(8000);
    }
    isJoined = lora.join();
    joinCount++;
    ack(60);
    delay(100);
    ack(60);
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
    delayIfSerial();
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
  powerDown();
#if LORAWAN
  waitForAck = false;
#endif
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
//------------------------------------------------------------sleep forever
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
  Serial.print("sleepcounter:");
  Serial.print(counter);
  Serial.print("/");
  Serial.println(int(ceil(sleepTime * 7.5)));
  delayIfSerial();
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
  upTime = millis();
  button.tick();

#if SERIAL
  Serial.println("Powering up");
  delayIfSerial();
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
    woke = true;
    gatherData();
  }

  //------------------------------------------------------------calibrate scale
#if SCALE
  if (calib && longPressed) {
    calibrate();
  }
#endif

  //----------------------------------------------------------LoRa RX timeout
#if LORA

  // Checks if the time since the last received packet is more than 600 milliseconds,
  // the device is awake (variable woke is true), and no packet has been received yet
  if (millis() - rxTime > 600 && woke && !isReceived) {
    isReceived = true;

#if SERIAL
    Serial.println("Gateway not answering.");
#endif

#if ACK_BEEP
    ack(10);
#endif

    // Checks if the number of transmission tries is less than the defined maximum
    if (nrOfTries < TX_RETRIES) {
      nrOfTries++;                      // Increments the number of transmission tries
      triedOnce = true;                 // prevents the message counter from counting
      counter = ceil(sleepTime * 7.5);  // sleep for 8 seconds and try again
    }

    // Puts the LoRa node to sleep
    sleepLoraNode();
  }
#endif


//----------------------------------------------------------LoRaWAN RX
#if LORAWAN
  if (woke && waitForAck) {
    // Check Lora RX
    lora.update();
    if (lora.readAck()) {
#if SERIAL
      Serial.println("LoRaWAN - TxDone");
      delayIfSerial();
#endif
      sleep();
    } else if (millis() - rxTime > 1000) {
#if SERIAL
      Serial.println("LoRaWAN - timeout");
      delayIfSerial();
#endif
      sleep();
    }
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
  sensorTime = millis();
  while (ds.selectNext()) {
    if (cTemp = 1) {
      tempC1 = ds.getTempC();
    } else {
      tempC2 = ds.getTempC();
    }
    cTemp++;
    if (millis() - sensorTime > 1000) break;
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
#if SERIAL
  Serial.println("...waiting for scale");
#endif
  sensorTime = millis();
  do {
    sensReady = myScale.is_ready();
    delay(1);
#if SERIAL
    Serial.print(".");
#endif
    if (millis() - sensorTime > 500) break;
  } while (!sensReady);

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
    Serial.println("Scale was not ready!");
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
      if (millis() - sensorTime > 300) break;
    } while (!sensReady) {*/
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
  //                                                      LoRA
#if LORA

#if SERIAL
  Serial.print("Sending packet (LoRa)...");
  Serial.println(Pcounter);
#endif
  payload = makePayload();
  LoRa_sendMessage(payload);  // send a message

#endif
  //                                                      LoRaWAN
#if LORAWAN
  sendToLorawan();
#endif
  //------

#if !LORA && !LORAWAN
  sleep();
#endif
}

#endif  //#if !LORA_GATEWAY


//***********************************************************************************************
//                                                                                  Make payload
//***********************************************************************************************

#if LORA || LORAWAN

String makePayload() {
  String myStr = "";
  checkStr = "";  //consists of counter+node number to address the node for acknowledgement

#if LORA
  if (!triedOnce) {  //if gateway was busy and a resend happend after 8 secs do not count
    nrOfMsgs++;
  }
#endif

  checkStr = String(nrOfMsgs) + "," + String(NODE_NR);
  myStr = checkStr + ",";
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

  myStr += String(millis() - upTime) + ",";

#if LORA || LORA_GATEWAY
  myStr += String(COMMON_PHRASE);
#endif

  Serial.println(payload);

  return myStr;
}

#endif
//***********************************************************************************************
//                                                                                  Send LoRaWAN
//***********************************************************************************************

#if LORAWAN

void sendToLorawan() {
#if SERIAL
  Serial.print("Sending packet (LoRaWan)...");
#endif
  String payload = makePayload();
  const char* payloadCStr = payload.c_str();
  size_t payloadLength = strlen(payloadCStr);

#if SERIAL
  Serial.print("Sending: ");
  Serial.println(payloadCStr);
#endif
  //sending
  lora.sendUplink(payloadCStr, payloadLength, 0, 1);

#if ACK_BEEP
  ack(10);
#endif
  rxTime = millis();
  waitForAck = true;
}

#endif

//***********************************************************************************************
//                                                                                     Send LoRa
//***********************************************************************************************

#if LORA

void LoRa_rxMode() {

  isReceived = false;  //starts the rx timout check
  rxTime = millis();

  LoRa.enableInvertIQ();  // active invert I and Q signals
  LoRa.receive();         // set receive mode
}

void LoRa_txMode() {
  LoRa.idle();             // set standby mode
  LoRa.disableInvertIQ();  // normal mode
}

void LoRa_sendMessage(String message) {
  LoRa_txMode();  // set tx mode

#if SERIAL
  Serial.print("Sending: ");
  Serial.println(message);
#endif

#if ENCRYPTION
  message = encryptData(message);
#endif
  LoRa.beginPacket();    // start packet
  LoRa.print(message);   // add payload
  LoRa.endPacket(true);  // finish packet and send it
}

void onReceive(int packetSize) {
  isReceived = true;  //stops the rx timout check
  String message = "";
  sensorTime = millis();
  while (LoRa.available()) {
    message += (char)LoRa.read();
    if (millis() - sensorTime > 500) break;
  }
#if ENCRYPTION
  message = decryptData(message);
#endif

#if SERIAL
  Serial.print("Received from Gateway: ");
  Serial.print(message);
  Serial.print(" | should be: ");
  Serial.println(checkStr);
  delayIfSerial();
#endif

  if (message == checkStr) {

#if ACK_BEEP
    ack(10000);
#endif

#if SERIAL
    Serial.println("The gateway has confirmed..");
    delayIfSerial();
#endif
    nrOfTries = 0;      //set back the counter for rx timeout retries
    triedOnce = false;  //count again;
    sleepLoraNode();
  } else {
#if SERIAL
    Serial.print("Gateway busy. Trying again in 8s...  ");
    delayIfSerial();
#endif
#if ACK_BEEP
    ack(1000);
#endif
    triedOnce = true;                 // prevents the message counter from counting
    counter = ceil(sleepTime * 7.5);  //sleep for 8 seconds and try again
    sleepLoraNode();
  }
}


void onTxDone() {
#if ACK_BEEP
  //ack(10);
#endif
#if SERIAL
  Serial.println("TxDone");
  Serial.println("wait for an answer...");
  delayIfSerial();
#endif
  LoRa_rxMode();
}

#endif


//***********************************************************************************************
//                                                                                  NODE sleep
//***********************************************************************************************

#if LORA && !LORA_GATEWAY

void sleepLoraNode() {
  LoRa_txMode();
  isReceived = true;
#if SERIAL
  Serial.print("Uptime:");
  Serial.println(millis() - upTime);
  Serial.println("...going to sleep");
  delayIfSerial();
#endif
#if ACK_BEEP
  //ack(10);
  //ack(10);  //when an acknowledgement is received go to sleep
#endif
  sleep();
}

#endif

//***********************************************************************************************
//                                                                                  LoRA Gateway
//***********************************************************************************************

#if LORA_GATEWAY

void onReceive(int packetSize) {

  String message = "";
  for (int i = 0; i < packetSize; i++) {
    char c = (char)LoRa.read();
    message += c;
  }

#if ENCRYPTION
  message = decryptData(message);
#endif

  if (message.substring(message.lastIndexOf(',') + 1).equals(COMMON_PHRASE)) {
    // Remove the common phrase including the comma
    message = message.substring(0, message.lastIndexOf(','));
    message += "," + String(LoRa.packetRssi()) + "," + String(LoRa.packetSnr());
    Serial.println(message);
    //send the first part (before the second comma) of the message back as an acknolegement ("counter,NODE_NAME")
    LoRa_sendMessage(message.substring(0, message.indexOf(',', message.indexOf(',') + 1)));
  }
}

void onTxDone() {
  LoRa_rxMode();
}

void LoRa_rxMode() {
  LoRa.disableInvertIQ();  // normal mode
  LoRa.receive();          // set receive mode
}

void LoRa_txMode() {
  LoRa.idle();            // set standby mode
  LoRa.enableInvertIQ();  // active invert I and Q signals
}

void LoRa_sendMessage(String message) {
  LoRa_txMode();  // set tx mode
#if ENCRYPTION
  message = encryptData(message);
#endif
  LoRa.beginPacket();    // start packet
  LoRa.print(message);   // add payload
  LoRa.endPacket(true);  // finish packet and send it
}

#endif



//***********************************************************************************************
//                                                                                           AES
//***********************************************************************************************

#if ENCRYPTION

String addPadding(const String& plaintext) {
  String paddedText = plaintext;
  int paddingLength = 16 - (plaintext.length() % 16);
  for (int i = 0; i < paddingLength; ++i) {
    paddedText += (char)paddingLength;
  }
  return paddedText;
}

String removePadding(const String& paddedText) {
  int paddingLength = paddedText[paddedText.length() - 1];
  return paddedText.substring(0, paddedText.length() - paddingLength);
}

String encryptData(const String& plaintext) {
  // Add padding to the plaintext
  String paddedText = addPadding(plaintext);

  // Convert the plaintext string to a byte array
  uint8_t* plaintextBytes = (uint8_t*)paddedText.c_str();
  uint16_t plaintextLength = paddedText.length();

  // Encrypt the padded plaintext using AES-128
  aes128_enc_multiple(key, plaintextBytes, plaintextLength);

  // Encode the encrypted byte array to Base64
  int encodedLength = Base64.encodedLength(plaintextLength);
  char encodedString[encodedLength + 1];
  Base64.encode(encodedString, plaintextBytes, plaintextLength);

  // Return the Base64 encoded string
  return String(encodedString);
}

String decryptData(const String& encryptedData) {
  // Decode the Base64 encoded string directly into a byte array
  int inputStringLength = encryptedData.length();
  int decodedLength = Base64.decodedLength(encryptedData.c_str(), inputStringLength);
  uint8_t decodedBytes[decodedLength];
  Base64.decode((char*)decodedBytes, encryptedData.c_str(), inputStringLength);

  // Decrypt the decoded byte array using AES-128
  aes128_dec_multiple(key, decodedBytes, decodedLength);

  // Convert the decrypted byte array to a string
  String decryptedData = "";
  for (int i = 0; i < decodedLength; i++) {
    decryptedData += (char)decodedBytes[i];
  }

  // Remove padding from the decrypted data
  decryptedData = removePadding(decryptedData);

  return decryptedData;
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

#if LORA && !LORA_GATEWAY
  triedOnce = false;
#endif

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
  delayIfSerial();
#endif

  digitalWrite(BUZZER_PIN, LOW);
  delay(t);
  digitalWrite(BUZZER_PIN, HIGH);
}


//***********************************************************************************************
//                                                                                         Delay
//***********************************************************************************************

#if SERIAL
void delayIfSerial() {
  delay(100);
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

      blinker(6);
      calib = false;
    }
  } else {

#if SERIAL
    Serial.println("Only set new tare");
    delayIfSerial();
#endif

    EEPROM.put(12, weight);
    ack(100);
  }
  longPressed = false;
  sleep();
}

#endif