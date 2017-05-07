//#define ALLEVENTS

#include <CurieBLE.h>

//CSC = Cycling Speed and Cadence
#define FLAGS_SIZE 1
#define WHEEL_REV_SIZE 4
#define CRANK_REV_SIZE 2
#define TIME_EVENT_SIZE 2
#ifdef ALLEVENTS
#define CSCMSIZE FLAGS_SIZE+WHEEL_REV_SIZE+TIME_EVENT_SIZE+CRANK_REV_SIZE+TIME_EVENT_SIZE
#else
#define CSCMSIZE FLAGS_SIZE+WHEEL_REV_SIZE+TIME_EVENT_SIZE
#endif

#define CSC_WHEEL_REV_PRESENT 1
#define CSC_CRANK_REV_PRESENT 2

#define MINTERVAL 3000

#define INTERRUPTPIN  2

BLEPeripheral blePeripheral;       // BLE Peripheral Device (the board you're programming)
BLEService cscService("1816"); // BLE CSCe Service

BLECharacteristic cscFeaturesChar ("2A5C", BLERead, 2); //16bit
BLECharacteristic cscMeasureChar ("2A5B",  BLENotify, CSCMSIZE); //bleread not needed
BLECharCharacteristic cscSensLocChar ("2A5D",  BLERead);
BLECharacteristic cscSCControlPointChar ("2A55",  BLEWrite | BLEIndicate, 255);

//BLEService devInfoService("180A"); // BLE CSCe Service
//BLECharacteristic manuNameChar ("2A29", BLERead, 3);
//docs see https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.service.cycling_speed_and_cadence.xml

//Services
// Cycling Speed and Cadence  0x1816
//Characteristics
//CSC Feature 0x2A5C
//CSC Measurement 0x2A5B

volatile uint32_t currWheelPos = 0;  // last heart rate reading from analog input
boolean isodd = false;
uint16_t currCrankPos = 0;  // last heart rate reading from analog input
uint16_t wheelTime = 0;
long previousMillis = 0;  // last time the heart rate was checked, in ms

volatile byte slice=0;
volatile bool updating=false;

void setup() {
  pinMode(INTERRUPTPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPTPIN), do_wcount, FALLING);

  pinMode(LED_BUILTIN, OUTPUT);
  blinkLED(3);
  Serial.begin(9600);    // initialize serial communication
  Serial.println(CSCMSIZE);
  /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet */
  blePeripheral.setLocalName("CSCSketch");
  blePeripheral.setAdvertisedServiceUuid(cscService.uuid());

  blePeripheral.addAttribute(cscService);

  //configure features chartecriscigtsc
#ifdef ALLEVENTS
  const unsigned char featFlagsArray[2] = {  CSC_CRANK_REV_PRESENT | CSC_WHEEL_REV_PRESENT, 0 };
#else
  const unsigned char featFlagsArray[2] = {  CSC_WHEEL_REV_PRESENT, 0 };
#endif

  blePeripheral.addAttribute(cscMeasureChar);

  blePeripheral.addAttribute(cscFeaturesChar);
  cscFeaturesChar.setValue(featFlagsArray, 2);

  blePeripheral.addAttribute(cscSensLocChar);
  cscSensLocChar.setValue(6);

  blePeripheral.addAttribute(cscSCControlPointChar);
  cscSCControlPointChar.setEventHandler(BLEWritten, cscSCControlPointCharWritten);

  //manu uinfo
  // blePeripheral.addAttribute(devInfoService);
  //const unsigned char manuArray[3] = { 0x41,0x42,0x43 };
  // blePeripheral.addAttribute(manuNameChar);
  // manuNameChar.setValue(manuArray,3);


  /* Now activate the BLE device.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */
  blePeripheral.begin();
  Serial.print("CSC Bluetooth device active, waiting for connections...");
#ifdef ALLEVENTS
  Serial.print("ALLEVENTS Mode ");
  Serial.println(CSCMSIZE);
#else
  Serial.print("ONLY wheel events Mode ");
  Serial.println(CSCMSIZE);
#endif
}

void loop() {
  // put your main code here, to run repeatedly:
  // listen for BLE peripherals to connect:
  BLECentral central = blePeripheral.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    // check the CSC measurement every 200ms
    // as long as the central is still connected:
    while (central.connected()) {
      if(updating){
        delay(1); //if value is currentliy updating wait a bit
      }
      long currentMillis = millis();
      // if MINTERVAL ms have passed, check the csc measurement:
      if (currentMillis - previousMillis >= MINTERVAL) {
        previousMillis = currentMillis;
        updateCSC(currentMillis) ;
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}


int oldRpm = 0;

void updateCSC(long currentMillis) {
  /* Read the current voltage level on the A0 analog input pin.
     This is used here to simulate the heart rate's measurement.
  */  
  unsigned long mltime = currentMillis % (65536);


  currCrankPos += 1; //fake

  unsigned char n24 = (unsigned char)(currWheelPos >> 24);
  unsigned char n16 = (unsigned char)(currWheelPos >> 16);
  unsigned char n8 = (unsigned char)(currWheelPos >> 8);
  unsigned char n0 = (unsigned char)currWheelPos;

  unsigned char m8 = (unsigned char)(mltime >> 8);
  unsigned char m0 = (unsigned char)mltime;

  unsigned char c8 = (unsigned char)(currCrankPos >> 8);
  unsigned char c0 = (unsigned char) currCrankPos;


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
  cscMeasureChar.setValue(rpmCharArray, CSCMSIZE);  // and update the heart rate measurement characteristic  
  
  //oldRpm = rpm;           // save the level for next comparison
  //}
}


void blinkLED(int cnt) {
  for (int i = 0; i < cnt; i++) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(200);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(300);
  }
}


void cscSCControlPointCharWritten(BLEDevice tcentral, BLECharacteristic tcharacteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print("Characteristic event, written: ");
  //
  //  if (switchChar.value()) {
  //    Serial.println("LED on");
  //    digitalWrite(ledPin, HIGH);
  //  } else {
  //    Serial.println("LED off");
  //    digitalWrite(ledPin, LOW);
  //  }
}

void do_wcount() {
  if (isodd) {
    isodd = false;
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    isodd = true;
  }
  if(slice>=2){
    slice=0;
    updating=true;
    currWheelPos += 1;
    updating=false;
  }else{
    slice+=1;
  }

  

}


