//#define ALLEVENTS
#define SERIALDBG
#define MINTERVAL 3100

#define FACTORYRESET_ENABLE
#define MINIMUM_FIRMWARE_VERSION   "0.7.0"

//bluefruit stuff
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BLEGatt.h"
#include "bluefruitconfig.h"


//conenience defines
#define ERROR_IF_ZERO(VARI,MSG) if(VARI==0) { error(F(MSG));  }
#define ERROR_IF_NOT(VARI,MSG) if(! VARI) { error(F(MSG));  }
#ifdef SERIALDBG
#define SERIALPRINT(VARI) Serial.print(VARI)
#define SERIALPRINTLN(VARI) Serial.println(VARI)
#else
#define SERIALPRINT(VARI) /*(VARI)*/
#define SERIALPRINTLN(VARI) /*(VARI)*/
#endif

volatile uint8_t *ledport;
volatile uint8_t ledbit;

#ifdef ALLEVENTS
const unsigned char featFlagsArray[2] = {  CSC_CRANK_REV_PRESENT | CSC_WHEEL_REV_PRESENT, 0 };
#else
const unsigned char featFlagsArray[2] = {  CSC_WHEEL_REV_PRESENT, 0 };
#endif
const unsigned char locationArray[1] = {  6 };
uint8_t advdataArray[9] = { 0x02, 0x01, 0x06, 0x05, 0x02, 0x09, 0x18, 0x0a, 0x18 };
#define ADVLOWIDPOS 5
#define ADVHIGHIDPOS 6

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BLEGatt gatt(ble);


// A small helper for showing error msg
void error(const __FlashStringHelper*err) {

  SERIALPRINT(err);

  while (1) {
    blinkLED(4);
    delay(1000);
  }
}

#ifdef ALLEVENTS
#define CSCMSIZE FLAGS_SIZE+WHEEL_REV_SIZE+TIME_EVENT_SIZE+CRANK_REV_SIZE+TIME_EVENT_SIZE
#else
#define CSCMSIZE FLAGS_SIZE+WHEEL_REV_SIZE+TIME_EVENT_SIZE
#endif


#define INTERRUPTPIN  2

int32_t cscServiceId;
int32_t cscMeasureCharId;
int32_t cscFeaturesCharId;
int32_t cscSensorLocCharId;
int32_t cscServicePointCharId;

volatile uint32_t currWheelPos = 0;  // last heart rate reading from analog input
boolean isodd = false;
uint16_t wheelTime = 0;
uint16_t currCrankPos = 0;  // last heart rate reading from analog input
long previousMillis = 0;  // last time the heart rate was checked, in ms

volatile byte slice = 0;
volatile bool updating = false;
volatile bool is_central_connected = false;
bool last_is_central_connected;

void setup() {
  initLEDPort();
  blinkLED(1);
#ifdef SERIALDBG
  Serial.begin(9600);    // initialize serial communication
#endif
  SERIALPRINT("Start: ");
  SERIALPRINT(__DATE__);
  SERIALPRINT(" ");
  SERIALPRINTLN(__TIME__);

  pinMode(INTERRUPTPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPTPIN), doWheelCount, FALLING);

  initBluetoothServices();

  pinMode(LED_BUILTIN, OUTPUT);
  blinkLED(3);

}

void loop() {
  ble.update(200);
  if (last_is_central_connected != is_central_connected) {
    if (is_central_connected) {
      SERIALPRINTLN( F("Connected") );
      readAndPrintPeerAddress();
    } else {
      SERIALPRINTLN( F("Disconnected") );
    }
    last_is_central_connected = is_central_connected;
  }
  if (is_central_connected)
  {
    long currentMillis = millis();
    // if MINTERVAL ms have passed, check the csc measurement:
    if (currentMillis - previousMillis >= MINTERVAL) {
      previousMillis = currentMillis;
      updateCSCviaBLE(currentMillis) ;
    }
  }
}

void initBluetoothServices() {
  boolean success;
  SERIALPRINTLN(F("CSC home biker peripheral"));
  /* Initialise the module */
  SERIALPRINT(F("Initialising the Bluefruit LE module: "));
  ERROR_IF_NOT( ble.begin(VERBOSE_MODE) , "Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?" );
  SERIALPRINTLN( F("OK!") );

#ifdef FACTORYRESET_ENABLE
  /* Perform a factory reset to make sure everything is in a known state */
  SERIALPRINTLN(F("Performing a factory reset: "));
  ERROR_IF_NOT( ble.factoryReset(), "Couldn't factory reset");
#endif
  ERROR_IF_NOT( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION), "Callback requires at firmware least 0.7.0" );

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  SERIALPRINTLN("Requesting Bluefruit info:");
  /* Print Bluefruit information */
#ifdef SERIALDBG
  ble.info();
#endif

  ble.setConnectCallback(central_connected);
  ble.setDisconnectCallback(central_disconnected);
  ble.setBleUartRxCallback(bleUartRX);

  ble.sendCommandCheckOK(F("AT+HWMODELED=1"));

  ERROR_IF_NOT( ble.sendCommandCheckOK(F("AT+GAPDEVNAME=INBIKE CSC")), "Could not set device name?" );

  cscServiceId = gatt.addService(GATT_CSC_SERVICE_UID);
  ERROR_IF_ZERO(cscServiceId, "Could not add CSC service")

  /* CSC Measurement characteristic */
  cscMeasureCharId = gatt.addCharacteristic(GATT_CSC_MEASURE_CHAR_UID, GATT_CHARS_PROPERTIES_NOTIFY, 2, CSCMSIZE, BLE_DATATYPE_BYTEARRAY);
  ERROR_IF_ZERO(cscMeasureCharId, "Failed adding csc rate measurment characteristic")

  cscFeaturesCharId = gatt.addCharacteristic(GATT_CSC_FEATURE_CHAR_UID, GATT_CHARS_PROPERTIES_READ, 2, 2, BLE_DATATYPE_BYTEARRAY);
  ERROR_IF_ZERO(cscFeaturesCharId, "Failed adding csc feature char");
  gatt.setChar(cscFeaturesCharId, featFlagsArray, 2);

  cscSensorLocCharId = gatt.addCharacteristic(GATT_CSC_SENSORLOC_CHAR_ID, GATT_CHARS_PROPERTIES_READ, 1, 1, BLE_DATATYPE_BYTEARRAY);
  ERROR_IF_ZERO(cscSensorLocCharId, "Failed adding sensor loc char");
  gatt.setChar(cscSensorLocCharId, locationArray, 1);

  cscServicePointCharId = gatt.addCharacteristic(GATT_CSC_CONTROL_POINT_CHAR_ID, GATT_CHARS_PROPERTIES_WRITE | GATT_CHARS_PROPERTIES_INDICATE, 1, 32, BLE_DATATYPE_BYTEARRAY);
  ERROR_IF_ZERO(cscServicePointCharId, "Failed adding control point char");


  /* Add the csc uuid to the advertising data (needed for Nordic apps to detect the service) */
  advdataArray[ADVLOWIDPOS] = GATT_CSC_SERVICE_UID_LOWBYTE;
  advdataArray[ADVHIGHIDPOS] = GATT_CSC_SERVICE_UID_HIGHBYTE;
  ble.setAdvData( advdataArray, sizeof(advdataArray) );

  //set connectable state for device
  ERROR_IF_NOT( ble.sendCommandCheckOK(F("AT+GAPCONNECTABLE=1")), "Could not set connectable mode" );

  //start adverising
  ERROR_IF_NOT( ble.sendCommandCheckOK(F("AT+GAPSTARTADV")), "Could not start adveristing" );

#ifdef ALLEVENTS
  SERIALPRINT("CSC BLUETOOTH ALLEVENTS Mode ");
  SERIALPRINTLN(CSCMSIZE);
#else
  SERIALPRINT("CSC BLUETOOTH ONLY wheel events Mode ");
  SERIALPRINTLN(CSCMSIZE);
#endif
  /* Reset the device for the new service setting changes to take effect */
  ble.reset();
}

void readAndPrintPeerAddress(void) {
  ble.println("AT+BLEGETPEERADDR");
  if (! ble.waitForOK() ) {
    SERIALPRINTLN(F("Failed to execute BLEGETPEERADDR"));
  }
  ble.readline();
}


void central_connected(void)
{
  is_central_connected = true;
}

void central_disconnected(void)
{
  is_central_connected = false;
}

void bleGattRX(int32_t chars_id, uint8_t data[], uint16_t len)
{
  SERIALPRINT( F("[BLE GATT RX] (" ) );
  SERIALPRINT(chars_id);
  SERIALPRINT(") ");

  if (chars_id == cscServicePointCharId)
  {
#ifdef SERIALDBG
    Serial.write(data, len);
    Serial.println();
#endif
  }
  //  else if (chars_id == charid_number)
  //  {
  //    int32_t val;
  //    memcpy(&val, data, len);
  //    Serial.println(val);
  //  }
}


void bleUartRX(char data[], uint16_t len)
{
#ifdef SERIALDBG
  Serial.print( F("[BLE UART RX]" ) );
  Serial.write(data, len);
  Serial.println();
#endif
}

void initLEDPort() {
  ledport = digitalPinToPort(LED_BUILTIN);
  ledbit = digitalPinToBitMask(LED_BUILTIN);
}

void blinkLED(int cnt) {
  for (int i = 0; i < cnt; i++) {
    (*ledport) = (*ledport) | ledbit ;
    delay(200);                       // wait for a second
    (*ledport) = (*ledport) & ~ledbit;
    delay(300);
  }
}

void setLED(boolean state) {
  if (state)
  {
    (*ledport) = (*ledport) | ledbit ;
  } else {
    (*ledport) = (*ledport) & ~ledbit;
  }

}


void updateCSCviaBLE(long currentMillis) {

  unsigned long mltime = currentMillis % (65536);

  currCrankPos += 3; //fake
  if (updating) {
    delay(1);
  }
  unsigned char n24 = (unsigned char)(currWheelPos >> 24);
  unsigned char n16 = (unsigned char)(currWheelPos >> 16);
  unsigned char n8 = (unsigned char)(currWheelPos >> 8);
  unsigned char n0 = (unsigned char)currWheelPos;

  unsigned char m8 = (unsigned char)(mltime >> 8);
  unsigned char m0 = (unsigned char)mltime;

  unsigned char c8 = (unsigned char)(currCrankPos >> 8);
  unsigned char c0 = (unsigned char) currCrankPos;

#ifdef SERIALDBG
  Serial.print(mltime);
  Serial.print(" ");
  Serial.print(m8);
  Serial.print(" ");
  Serial.print(m0);
  Serial.print(" w ");
  Serial.print(n24);
  Serial.print(" ");
  Serial.print(n16);
  Serial.print(" ");
  Serial.print(n8);
  Serial.print(" ");
  Serial.print(n0);
  Serial.print(" c ");
  Serial.print(c8);
  Serial.print(" ");
  Serial.print(c0);
  Serial.println();
#endif

#ifdef ALLEVENTS
  const unsigned char rpmCharArray[CSCMSIZE] = { CSC_WHEEL_REV_PRESENT | CSC_CRANK_REV_PRESENT  ,
                                                 n0, n8, n16, n24, m0, m8,
                                                 c0, c8, m0, m8
                                               };
#else
  const unsigned char rpmCharArray[CSCMSIZE] = { CSC_WHEEL_REV_PRESENT ,
                                                 n0, n8, n16, n24, m0, m8
                                               };
#endif
  gatt.setChar(cscMeasureCharId, rpmCharArray, CSCMSIZE);

}


void doWheelCount() {
  if (isodd) {
    isodd = false;
  } else {
    isodd = true;
  }
  if (slice >= 4) {
    slice = 0;
    updating = true;
    currWheelPos += 1;
    updating = false;
  } else {
    slice += 1;
  }

  setLED(isodd);


}



