//Arduino NANO 33 BLE Sense: Tempeature and Pressure Sensor. Reference: https://ladvien.com/arduino-nano-33-bluetooth-low-energy-setup/
// https://rootsaid.com/arduino-ble-example/
// Characteristic info.
// https://www.arduino.cc/en/Reference/ArduinoBLEBLECharacteristicBLECharacteristic

#include <ArduinoBLE.h>
#include <Wire.h>
#include <Arduino_HTS221.h>
#include <Arduino_APDS9960.h>
#include <Arduino_LPS22HB.h>




int color = 0;

// This device's MAC:
// C8:5C:A2:2B:61:86
#define LEDR        (23u)
#define LEDG        (22u)
#define LEDB        (24u)

// Device name
const char* nameOfPeripheral = "MicrophoneMonitor";
const char* uuidOfService = "00001101-0000-1000-8000-00805f9b34fb";
const char* uuidOfRxChar = "00001142-0000-1000-8000-00805f9b34fb";
const char* uuidOfTxChar = "00001143-0000-1000-8000-00805f9b34fb";
//const char* uuidOfTxFloat = "00001144-0000-1000-8000-00805f9b34fb";

// BLE Service
BLEService microphoneService(uuidOfService);

// Setup the incoming data characteristic (RX).
const int WRITE_BUFFER_SIZE = 256;
bool WRITE_BUFFER_FIZED_LENGTH = false;

// RX / TX Characteristics
BLECharacteristic rxChar(uuidOfRxChar, BLEWriteWithoutResponse | BLEWrite, WRITE_BUFFER_SIZE, WRITE_BUFFER_FIZED_LENGTH);
BLECharacteristic txChar(uuidOfTxChar, BLERead | BLENotify | BLEBroadcast, WRITE_BUFFER_SIZE, WRITE_BUFFER_FIZED_LENGTH);
//BLEFloatCharacteristic txFloat(uuidOfTxFloat, BLERead | BLENotify | BLEBroadcast, WRITE_BUFFER_SIZE, WRITE_BUFFER_FIZED_LENGTH);
// Buffer to read samples into, each sample is 16-bits
short sampleBuffer[256];

// Number of samples read
volatile int samplesRead;

//--------------

//void onPDMdata() {
//  // query the number of bytes available
//  int bytesAvailable = PDM.available();
//  Serial.println("PDM available");
//
//  // read into the sample buffer
//  PDM.read(sampleBuffer, bytesAvailable);
//
//  // 16-bit, 2 bytes per sample
//  samplesRead = bytesAvailable / 2;
//}

//------------
void startBLE() {
  Serial.println("startBLE");
  if (!BLE.begin())
  {
    Serial.println("starting BLE failed!");
    while (1);
  }
}

//----------------------

void connectedLight() {
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, HIGH);
}

//------
void disconnectedLight() {
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, LOW);
}

//---------------------

void onBLEConnected(BLEDevice central) {
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
  connectedLight();
}

void onBLEDisconnected(BLEDevice central) {
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
  disconnectedLight();
}

//-------------



void onRxCharValueUpdate(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print("Characteristic event, read: ");
  byte test[256];
  int dataLength = rxChar.readValue(test, 256);

  for(int i = 0; i < dataLength; i++) {
    Serial.print((char)test[i]);
  }//for
  Serial.println();
  Serial.print("Value length = ");
  Serial.println(rxChar.valueLength());
 
}


#include <stdio.h>

char *dtostrf (double val, signed char width, unsigned char prec, char *sout) {
  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;
}



void onTxCharValueUpdate(BLEDevice central, BLECharacteristic characteristic) {
  Serial.println("TxCharValueUpdate");
//--------
  //while (!Serial);

  if (!HTS.begin()) {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1);
  }

   if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    while (1);

   }
   

  float temperature = HTS.readTemperature();
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" °C");

  float pressure = BARO.readPressure();
  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" kPa");


//----------------------------------------------------------------
//#include <dtostrf.h>
 
char tempstring[15];
char presstring[15];
char finalstring[15];


dtostrf(temperature, 5, 2, tempstring);
dtostrf(pressure, 6, 2, presstring);

String stringOne = " ";
stringOne += tempstring;
stringOne += " and ";
stringOne += presstring;

stringOne.toCharArray(finalstring, 15);

txChar.writeValue(finalstring); 

}

//void char update

//-----------------------------------------------------------------


void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
while(!Serial);

Serial.println("Setting Up");

  if (!APDS.begin()) {
    Serial.println("Error initializing APDS9960 sensor.");
  }
    
pinMode(LED_BUILTIN, OUTPUT);
pinMode(LEDR, OUTPUT);
pinMode(LEDG, OUTPUT);



 // Configure the data receive callback


  // Start BLE.
startBLE();

 BLE.setLocalName(nameOfPeripheral);
  BLE.setAdvertisedService(microphoneService);
  microphoneService.addCharacteristic(rxChar);
  microphoneService.addCharacteristic(txChar);
  BLE.addService(microphoneService);

  // Bluetooth LE connection handlers.
  BLE.setEventHandler(BLEConnected, onBLEConnected);
  BLE.setEventHandler(BLEDisconnected, onBLEDisconnected);
  
  // Event driven reads.
  rxChar.setEventHandler(BLEWritten, onRxCharValueUpdate);
  txChar.setEventHandler(BLERead, onTxCharValueUpdate);
//   rxChar.setEventHandler(BLEWritten, onRxReply);
  
  // Let's tell devices about us.
  BLE.advertise();
  
  // Print out full UUID and MAC address.
  Serial.println("Peripheral advertising info: ");
  Serial.print("Name: ");
  Serial.println(nameOfPeripheral);
  Serial.print("MAC: ");
  Serial.println(BLE.address());
  Serial.print("Service UUID: ");
  Serial.println(microphoneService.uuid());
  Serial.print("rxCharacteristic UUID: ");
  Serial.println(uuidOfRxChar);
  Serial.print("txCharacteristics UUID: ");
  Serial.println(uuidOfTxChar);
  

  Serial.println("Bluetooth device active, waiting for connections...");
}








void loop() {
  // put your main code here, to run repeatedly:
Serial.println("loop");
delay(1000);

//----------------

BLEDevice central = BLE.central();
  
  if (central) {
    while (central.connected()) {
      connectedLight();
      
    //  txChar.writeValue(1);      
    
          }//while
  }//if central

   
  else {
    disconnectedLight();
   }//else

 txChar.writeValue("hello");
 //------------



}//loop
