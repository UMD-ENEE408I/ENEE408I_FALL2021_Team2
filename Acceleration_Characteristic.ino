#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

BLEService motorService("0fe79935-cd39-480a-8a44-06b70f36f248");
BLEService accelerationService ("2e0b7a48-510d-4779-abc9-d4b52067341c");

// create switch characteristic and allow remote device to read and write
BLEUnsignedIntCharacteristic accelerationCharacteristic("2e0b7a48-510d-4779-abc9-d4b52067341c", BLERead | BLEWrite);
  float a;
  
BLEUnsignedIntCharacteristic motorCharacteristic("0fe79935-cd39-480a-8a44-06b70f36f249", BLERead | BLEWrite);
  float x;
  float y;
  float z;
void setup() {
  Serial.begin(115200);

  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  // Set the connection interval to be as fast as possible (about 40 Hz)
  BLE.setConnectionInterval(0x0006, 0x0006);

  BLE.setLocalName("Brian_mouse");
  BLE.setAdvertisedService(motorService);
  motorService.addCharacteristic(motorCharacteristic);
  BLE.addService(motorService);
  
  BLE.setAdvertisedService(accelerationService);
  accelerationService.addCharacteristic(accelerationService);
  BLE.addService(accelerationService);

  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // assign event handlers for characteristic
  motorCharacteristic.setEventHandler(BLEWritten, motorCharacteristicWritten);
  motorCharacteristic.setValue(0);

  accelerationCharacteristic.setEventHandler(BLEWritten, accelerationCharacteristicWritten);
  accelerationCharacteritic.setValue(100);

  BLE.advertise();
  Serial.println("Waiting for connection");
}

void loop() {
  BLE.poll(100);
  IMU.readAcceleration(x,y,z);
  Serial.print(x);
  Serial.print('\t');
  Serial.print(y);
  Serial.print('\t');  
  motorCharacteristic.writeValue(100);
  accelerationCharacteristic.writeValue(200);
}

void blePeripheralConnectHandler(BLEDevice central) {
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}

void motorCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("motorCharacteristicWritten: ");
  unsigned int v = motorCharacteristic.value();
  short left = (short)((v>>0) & 0x0000FFFF); // Unpack 16 bit signed value (assume short is 16 bit)
  short right = (short)((v>>16) & 0x0000FFFF); // Unpack 16 bit signed value
  Serial.print(left);
  Serial.print(" ");
  Serial.println(right);
}

void accelerationCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("AccelerationCharacteristicWritten: ");
  unsigned int k = accelerationCharacteristic.value();
  short v = (short)((k>>0) & 0x0000FFFF);
  Serial.print(v);
  Serial.print(" ");
}
