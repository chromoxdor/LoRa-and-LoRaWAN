//**********************************************************************************************
//************* OPTIONS ************************************************************************

// Common Setting Section ---------------------------------------------------------------
#define LORA_FREQUENCY 868E6   // Frequency in Hz (433E6, 868E6, 915E6)
#define NODE_NR "2"            // Give your node a number or a name

// LoRa Section ------------------------------------------------------------------------
#define LORA 0                  // Is LoRa enabled? (0 or 1)
#define TX_RETRIES 2            // How often should the LoRa node try to send when no ACK is received until
                                // the system goes to sleep again (interval for retry is 8 seconds).
                                // It will then return to its given "sleep-time" interval.
#define LORA_GATEWAY 0          // Is LoRa Gateway enabled? (0 or 1)

#define ENCRYPTION 1            // Add simple AES128 encryption to LoRa
#define PASSWORD "XXXXXXXXXXXXXXXXXX" // Enter a password and make sure it matches the one of
                                       // the gateway/node
#define COMMON_PHRASE "CXD"     // Gateway checks if received message contains this phrase
                                // at the end and only reacts to that
#define MAX_PAYLOAD_SIZE 50     // Define maximum payload size

// LoRaWAN Section ----------------------------------------------------------------------
// NOTE: Define your maximum payload size for LoRaWAN (MAX_UPLINK_PAYLOAD_SIZE)
// in the config.h of the LoRaWAN library
#define LORAWAN 1
#define DEVEUI "0000000000000000"
#define APPEUI "0000000000000000"
#define APPKEY "00000000000000000000000000000000"
#define JOIN_RETRY 100          // How often should the node try to join a gateway until it will go
                                // to sleep indefinitely

// Sensor Section ------------------------------------------------------------------------
#define SCALE 0
#define CALIBWEIGHT 1000        // Weight used for calibrating the scale in grams

#define SHT 1                    // Use an SHT2X I2C device

#define DS18B20_TEMP 1          // Set to 1 if one or multiple sensors are connected to one pin
#define DS18B20_TEMP2 0         // Set to 1 if one or two sensors (on different pins) are connected

// Extras Section ------------------------------------------------------------------------
#define SLEEP_TIME 0            // Sleep time in minutes (setting the sleep time to 0 will cause an 8-second interval)
#define ACK_BEEP 0              // If an acknowledge beep is needed when data was sent
#define SERIAL 1                // Save some memory if not needed since it is only for debugging
#define LONGPRESS_S 5           // Seconds to long press event
#define SERIAL_SPEED 115200     // Baud rate (e.g., 9600, 19200, 38400, 57600, 115200)
#define PARITY SERIAL_8N1       // Set serial mode (e.g., SERIAL_8N1, SERIAL_8E1, SERIAL_8O1)
//**********************************************************************************************
//**********************************************************************************************


#if LORA_GATEWAY
#undef LORAWAN
#undef LORA
#undef SCALE
#undef SHT
#undef DS18B20_TEMP
#undef DS18B20_TEMP2
#endif

#if LORA
bool isReceived = true;
bool triedOnce = false;
int nrOfTries = 0;
unsigned long nrOfMsgs;
String checkStr;
#undef LORAWAN
#endif

#if LORA || LORAWAN
unsigned long rxTime;
#endif

#if LORA || LORA_GATEWAY
#include "SPI.h"
#include "LoRa.h"        //https://github.com/sandeepmistry/arduino-LoRa
const int csPin = 10;    // LoRa radio chip select
const int resetPin = 9;  // LoRa radio reset
const int irqPin = 3;    // change for your board; must be a hardware interrupt pin
#endif

#if LORAWAN
#undef ENCRYPTION
#undef SERIAL

#include <lorawan.h>  //https://github.com/ElectronicCats/Beelan-LoRaWAN

// OTAA credentials
const char* devEui = DEVEUI;
const char* appEui = APPEUI;
const char* appKey = APPKEY;

const sRFM_pins RFM_pins = {
  .CS = 10,
  .RST = 9,
  .DIO0 = 3,
  .DIO1 = 4,
  .DIO2 = 5,
  .DIO5 = -1,
};

int joinCount = 0;
bool isJoined = false;
bool once = false;
bool runOnce = false;
bool waitForAck = false;
#endif

#if SCALE
#include <GyverHX711.h>  //https://github.com/GyverLibs/GyverHX711
GyverHX711 myScale(A2, A1, HX_GAIN64_A);
// HX_GAIN128_A - канал А усиление 128
// HX_GAIN32_B - канал B усиление 32
// HX_GAIN64_A - канал А усиление

float scale;
long tare;
uint32_t offset;
long weight;
unsigned long currentCalibTime = 0;
#endif

#if SHT
#undef SCALE
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

//change the lenght of payload to accommodate encrytion
#define AES_BLOCK_SIZE 16  // AES block size in bytes
#define MAX_PAYLOAD_SIZE_ENCODED (((MAX_PAYLOAD_SIZE / AES_BLOCK_SIZE) + 1) * AES_BLOCK_SIZE * 4/3)
#endif

#include <EEPROM.h>
#include "LowPower.h"  //https://github.com/rocketscream/Low-Power

#if !LORAWAN

#if ENCRYPTION
char payload[MAX_PAYLOAD_SIZE_ENCODED];
#else
char payload[MAX_PAYLOAD_SIZE];
#endif

#else 
char payload[MAX_UPLINK_PAYLOAD_SIZE]; //
#endif 

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




//***********************************************************************************************
//                                                                                        Setup
//***********************************************************************************************

void setup() {
  pinMode(MOSFET, OUTPUT);
  digitalWrite(MOSFET, HIGH);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

#if ACK_BEEP
  ack(200);
#endif

#if SERIAL
  Serial.begin(SERIAL_SPEED, PARITY);
#endif

//------------------------------------------------------------scale init

#if SCALE
  EEPROM.get(1, offset);
  EEPROM.get(15, scale);
  if (isnan(scale)) {
    scale = 0;
  }
  if (isnan(offset)) {
    offset = 0;
  }
#if SERIAL
  Serial.print("\noffset(");
  Serial.print(offset);
  Serial.print("); scale(");
  Serial.print(scale, 6);
  Serial.println(");\n");
  Serial.print("); tare(");
  Serial.print(tare);
  Serial.println(");\n");
#endif
#endif

//------------------------------------------------------------buttons init
#if SCALE

  button.attachLongPressStart(LongPressStart);
  button.setPressMs(LONGPRESS_S * 1000);

#endif

  button.attachClick(click);


  //------------------------------------------------------------encrytion key
#if ENCRYPTION
  convertPassphraseToKey(PASSWORD, key);
  Serial.print("Password: ");
  for (int i = 0; i < 16; ++i) {
    Serial.print((char)key[i]);
  }
  Serial.println();
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

#if SERIAL
  Serial.println("Starting LoRaWAN Node");
#endif

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
  joinCount = 0;
  do {
#if SERIAL
    Serial.println("Joining...");
    Serial.println(joinCount);
    delayIfSerial();
#endif
#if SCALE
    while (!digitalRead(BUTTON_PIN)) {
      if (!runOnce) {
        rxTime = millis();
        runOnce = true;
      }
      if (millis() - rxTime > 1500) {
        button.tick();
        calibrate();
      }
    }
    runOnce = false;
#endif
    joinCount++;
    ack(10);
    delay(100);
    ack(10);
    isJoined = lora.join();
    if (isJoined) break;
    delay(8000);
  } while (joinCount < JOIN_RETRY);

  if (isJoined) {
    ack(500);
#if SERIAL
    Serial.println("Joined to network");
#endif
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

#if LORAWAN
  waitForAck = false;
  runOnce = false;
#endif

  if (woke) {
    powerDown();
  }
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
  //tryJoining();
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
  button.tick();

#if SERIAL
  Serial.println("Powering up");
  delayIfSerial();
#endif

#if SCALE
  myScale.sleepMode(false);
#endif

#if LORAWAN
  lora.wakeUp();
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

#if SCALE
  myScale.sleepMode(true);
#endif

#if LORAWAN
  lora.sleep();
#endif
}


//***********************************************************************************************
//                                                                                         RESET
//***********************************************************************************************

//used for rejoining the lorawan
void (*resetFunc)(void) = 0;  //declare reset function @ address 0


#endif  //#if !LORA_GATEWAY


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


#if LORAWAN  // for loranwan we need another approch for a button press \
             // since there was an issue with using the onebutton library for that

  if (!digitalRead(BUTTON_PIN)) {
    if (!runOnce) {
      rxTime = millis();
      runOnce = true;
    }
    if (millis() - rxTime > LONGPRESS_S*1000) {
      ack(100);
      resetFunc();
    }
  } else {
    if (runOnce) {
      ack(100);
      woke = true;
      gatherData();
    }
  }
#else
  if (clicked && !calib) {
    ack(100);
    clicked = false;
    woke = true;
    gatherData();
  }
#endif

  //-----------------------------------------------------call calibrate scale
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

    if (once) {
      once = false;
      // lora.sendUplink(payload, strlen(payload), 0, 1);
      rxTime = millis();
    }
    if (millis() - rxTime > 100) {
#if ACK_BEEP
      ack(10);
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

  upTime = millis();
  powerUp();
  memset(payload, 0, sizeof(payload));  // Clear payload buffer


#if LORA
  if (!triedOnce) {  //if gateway was busy and a resend happend after 8 secs do not count
    nrOfMsgs++;
  }
  // Convert unsigned long to string
  char nrOfMsgsStr[20];              // Assuming a maximum of 20 characters for an unsigned long
  ultoa(nrOfMsgs, nrOfMsgsStr, 10);  // 10 for decimal base

  // Concatenate strings
  strcat(payload, nrOfMsgsStr);
  strcat(payload, ",");
  strcat(payload, NODE_NR);
  strcat(payload, ",");

  checkStr = "";  //consists of counter + node number to address the node for acknowledgement
  checkStr = String(nrOfMsgs) + "," + String(NODE_NR);

#if SERIAL
  Serial.print("checkStr:");
  Serial.println(checkStr);
#endif

#endif  //LORA

  //------------------------------------------------------------battery
  char volt[4];
  dtostrf(analogRead(A0) * (4.2 / 900.0), 3, 2, volt);
  strcat(payload, volt);
  strcat(payload, ",");

#if SERIAL
  Serial.print("sensorValue:");
  Serial.println(analogRead(A0));
  Serial.print("Voltage:");
  Serial.println(analogRead(A0) * (4.2 / 900.0));
#endif

  //------------------------------------------------------------ds18b20
#if DS18B20_TEMP
  char tempBuffer[10];
  sensorTime = millis();
  while (ds.selectNext()) {
    dtostrf(ds.getTempC(), 3, 1, tempBuffer);
    strcat(payload, tempBuffer);
    strcat(payload, ",");
    if (millis() - sensorTime > 1000) break;
  }
#endif  //DS18B20_TEMP

#if DS18B20_TEMP2

  char tempBuffer[10];
  sensor1.requestTemp();
  sensor2.requestTemp();
  if (sensor1.readTemp()) {
    dtostrf(sensor1.getTemp(), 3, 1, tempBuffer);
    strcat(payload, tempBuffer);
  } else {
    dtostrf(-127.0, 3, 1, tempBuffer);
    strcat(payload, tempBuffer);
  }

#if SERIAL
  Serial.print("temp 1:");
  Serial.println(tempBuffer);
#endif

  strcat(payload, ",");

  if (sensor2.readTemp()) {
    //tempC2 = sensor2.getTemp();
    dtostrf(sensor2.getTemp(), 3, 1, tempBuffer);
    strcat(payload, tempBuffer);
  } else {
    dtostrf(-127.0, 3, 1, tempBuffer);
    strcat(payload, tempBuffer);
  }

#if SERIAL
  Serial.print("temp 2:");
  Serial.println(tempBuffer);
#endif

  strcat(payload, ",");

#endif  //DS18B20_TEMP2

  //------------------------------------------------------------SHT
#if SHT

  char shtBuffer[10];
  //sensorTime = millis();
#if SERIAL
  Serial.println("...waiting for SHT");
#endif
  /*do {
      sensReady = sensor.measure();
      delay(1);
      if (millis() - sensorTime > 300) break;
    } while (!sensReady) {*/
  //tempSHT = sht.getTemperature(65);
  //humSHT = sht.getHumidity(28);

  dtostrf(sht.getTemperature(65), 3, 1, tempBuffer);
  strcat(payload, tempBuffer);
  strcat(payload, ",");
  dtostrf(sht.getHumidity(28), 2, 0, tempBuffer);
  strcat(payload, tempBuffer);
  strcat(payload, ",");

#if SERIAL
  Serial.print("Temperature (°C): ");
  Serial.println(sht.getTemperature(65));
  Serial.print("Humidity (%RH): ");
  Serial.println((int)sht.getHumidity(28));
#endif

#endif  //SHT

  //------------------------------------------------------------scale

#if SCALE
  char weightBuffer[12];
  bool sensReady = false;
  sensorTime = millis();
  do {
    sensReady = myScale.available();
    if (millis() - sensorTime > 800) break;
  } while (!sensReady);

  dtostrf((static_cast<float>(myScale.read()) - offset) * scale / 1000, 4, 2, weightBuffer);
  strcat(payload, weightBuffer);
  strcat(payload, ",");
  weightBuffer[0] = '\0';  // Clears the contents of weightBuffer

#endif  //SCALE

//-----------------------------------------------------add or remove stuff
#if LORA || LORA_GATEWAY
  strcat(payload, COMMON_PHRASE);
  payload[strlen(payload)] = '\0';
#endif

#if LORAWAN
  if (strlen(payload) > 0) {
    payload[strlen(payload) - 1] = '\0';
  }
#endif

#if SERIAL
  Serial.print("Payload:");
  Serial.println(payload);
#endif

  //------------------------------------------------------------send data

  //                                                      LoRA
#if LORA

#if SERIAL
  Serial.print("Sending packet (LoRa)...");
#endif

  LoRa_sendMessage(String(payload));  // send a message
#endif                                //LORA


  //                                                   LoRaWAN

#if LORAWAN
  lora.sendUplink(payload, strlen(payload), 0, 1);
  // once = true;
  // waitForAck = true;
  #if ACK_BEEP
      ack(10);
#endif
      sleep();
#endif  //LORAWAN


  //                                                      else

#if !LORA && !LORAWAN
  delay(100);
  sleep();
#endif
}
#endif  //#if !LORA_GATEWAY


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
      delay(400);
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
  myScale.sleepMode(false);
  longPressed = false;
  calib = true;
  blinker(2);
  bool sensReady = false;
  sensorTime = millis();
  do {
    sensReady = myScale.available();
    if (millis() - sensorTime > 800) break;
  } while (!sensReady);

  if (sensReady) {
    weight = myScale.read();
  }
#if SERIAL
  Serial.println("Tare scale...");
  Serial.print("weight:");
  Serial.println(weight);
#endif
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
      //Serial.print("WEIGHT: ");
      //Serial.println(weightCalib);
      float newWeight = myScale.read();
      scale = CALIBWEIGHT / (newWeight - weight);
#if SERIAL
      Serial.print("newweight:  ");
      Serial.println(newWeight);
#endif

      for (int i = 0; i < 30; i++) {
        EEPROM.write(i, 0);
      }
      myScale.setOffset(0);
      offset = weight,
      EEPROM.put(1, weight);
      EEPROM.put(15, scale);

#if SERIAL
      Serial.print("offset(");
      Serial.print(weight);
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
    for (int i = 0; i < 15; i++) {
      EEPROM.write(i, 0);
    }
    offset = weight;
    Serial.println(weight);
    EEPROM.put(1, weight);
    ack(100);
  }

  longPressed = false;
#if LORAWAN
  resetFunc();
#endif
  sleep();
}

#endif