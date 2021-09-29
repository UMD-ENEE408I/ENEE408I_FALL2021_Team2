#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

BLEService motorService("0fe79935-cd39-480a-8a44-06b70f36f248");
BLEService accelerationService ("2e0b7a48-510d-4779-abc9-d4b52067341c");
BLEService gyroService ("41d9d6f5-9932-427a-888e-86dd23bace6e");

// create switch characteristic and allow remote device to read and write
BLEFloatCharacteristic accelerationCharacteristic("2e0b7a48-510d-4779-abc9-d4b52067341c", BLERead | BLEWrite);
BLEFloatCharacteristic Accel2Char("2e0b7a48-510d-4779-abc9-d4b52067341d", BLERead | BLEWrite);

BLEUnsignedIntCharacteristic motorCharacteristic("0fe79935-cd39-480a-8a44-06b70f36f249", BLERead | BLEWrite);

BLEFloatCharacteristic gyroZChar("41d9d6f5-9932-427a-888e-86dd23bace6f", BLERead | BLEWrite);

float a;

float x;
float y;
float z;

float q;
float k;
float j;

void setup() {
  Serial.begin(115200);
  
  if (!IMU.begin()) {
    Serial.println("starting IMU failed!");
    while (1);
  }
  
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
  accelerationService.addCharacteristic(accelerationCharacteristic);
  BLE.addService(accelerationService);

  BLE.setAdvertisedService(accelerationService);
  accelerationService.addCharacteristic(Accel2Char);
  BLE.addService(accelerationService);

  BLE.setAdvertisedService(gyroService);
  accelerationService.addCharacteristic(gyroZChar);
  BLE.addService(gyroService);
  
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // assign event handlers for characteristic
  motorCharacteristic.setEventHandler(BLEWritten, motorCharacteristicWritten);
  motorCharacteristic.setValue(0);

  accelerationCharacteristic.setEventHandler(BLEWritten, accelerationCharacteristicWritten);
  accelerationCharacteristic.setValue(0);

  Accel2Char.setEventHandler(BLEWritten, Accel2CharWritten);
  accelerationCharacteristic.setValue(0);

  Accel2Char.setEventHandler(BLEWritten, gyroZWritten);
  accelerationCharacteristic.setValue(0);
  
  BLE.advertise();
  Serial.println("Waiting for connection");
}

void loop() {
  BLE.poll(100);
  
  if(IMU.accelerationAvailable()){
  IMU.readAcceleration(x,y,z);
  accelerationCharacteristic.writeValue(x);
  Accel2Char.writeValue(y);
  //Accel3Char.writeValue(z);
  motorCharacteristic.writeValue(200);
  }
  if(IMU.gyroscopeAvailable()){
    IMU.readGyroscope(q,k,j);
    gyroZChar.writeValue(j);
  }
  
  Serial.print(x);
  Serial.print('\t');
  Serial.print(y);
  Serial.print('\t');
  Serial.print(z);
  Serial.println('\t');
  
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
  Serial.print("Acceleration1CharacteristicWritten: ");
  unsigned int k = accelerationCharacteristic.value();
  short v = (short)((k>>0) & 0x0000FFFF);
  Serial.print(v);
  Serial.print(" ");
}

void Accel2CharWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("Acceleration2CharacteristicWritten: ");
  unsigned int k = Accel2Char.value();
  short v = (short)((k>>0) & 0x0000FFFF);
  Serial.print(v);
  Serial.print(" ");
}

void gyroZWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("GyroScopeZCharacteristicWritten: ");
  unsigned int k = gyroZChar.value();
  short v = (short)((k>>0) & 0x0000FFFF);
  Serial.print(v);
  Serial.print(" ");
}
