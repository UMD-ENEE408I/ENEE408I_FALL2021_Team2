/*
  This example creates a BLE peripheral with a service that contains a
  couple of characteristics to test BLE connection.
  The yellow LED shows the BLE module is initialized.
  The green LED shows RSSI of zero. The more it blinks the worse the connection.

  The circuit:
  - Arduino Nano 33 BLE Sense board.

  You can use a generic BLE central app, like LightBlue (iOS and Android) or
  nRF Connect (Android), to interact with the services and characteristics
  created in this sketch.

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>


//----------------------------------------------------------------------------------------------------------------------
// BLE UUIDs
//----------------------------------------------------------------------------------------------------------------------

#define BLE_UUID_TEST_SERVICE               "9A48ECBA-2E92-082F-C079-9E75AAE428B1"
#define BLE_UUID_ACCELERATION               "2713"
#define BLE_UUID_COUNTER                    "1A3AC130-31EE-758A-BC50-54A61958EF81"
#define BLE_UUID_STRING                    "1A3AC131-31EF-758B-BC51-54A61958EF82"
#define BLE_UUID_BUFFER                    "1A3AC132-31ED-758C-BC52-54A61958EF82"
#define BLE_UUID_RESET_COUNTER              "FE4E19FF-B132-0099-5E94-3FFB2CF07940"

//----------------------------------------------------------------------------------------------------------------------
// BLE
//----------------------------------------------------------------------------------------------------------------------



BLEService testService( BLE_UUID_TEST_SERVICE );
BLEFloatCharacteristic accelXCharacteristic( BLE_UUID_ACCELERATION, BLERead | BLEWrite );
BLEFloatCharacteristic accelYCharacteristic( BLE_UUID_COUNTER, BLERead | BLEWrite );
BLEFloatCharacteristic accelZCharacteristic( BLE_UUID_RESET_COUNTER, BLERead | BLEWrite );
//BLECharacteristic stringCharacteristic( BLE_UUID_STRING, BLERead | BLENotify,"TEST1" );
//BLECharacteristic buffCharacteristic( BLE_UUID_BUFFER, BLERead | BLENotify,20,(1==1) );


const int BLE_LED_PIN = LED_BUILTIN;
const int RSSI_LED_PIN = LED_PWR; 
char buf[20];

void setup()
{
  Serial.begin( 9600 );
  //  while ( !Serial );

  pinMode( BLE_LED_PIN, OUTPUT );
  pinMode( RSSI_LED_PIN, OUTPUT );
  
  
  Serial.print( "Accelerometer sample rate = " );
  Serial.print( 10 );
  Serial.println( " Hz" );

  if( setupBleMode() )
  {
    digitalWrite( BLE_LED_PIN, HIGH );
  }
} // setup


void loop()
{
  static unsigned long counter = 0;
  static long previousMillis = 0;
  uint8_t x;
  String aS ="TEst1String";
  
    float accelerationX =0, accelerationY, accelerationZ;
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  if ( central )
  {
    Serial.print( "Connected to central: " );
    Serial.println( central.address() );

    while ( central.connected() )
    {
      if( resetCounterCharacteristic.written() )
      {
        counter = 0;
        Serial.println( "Reset" );
        //Serial.println(resetCounterCharacteristic.value()  );
        stringCharacteristic.writeValue("test2222");
        aS.toCharArray(buf,20);
        buffCharacteristic.writeValue( buf,20 );

      }

      long interval = 5;
      unsigned long currentMillis = millis();
      if( currentMillis - previousMillis > interval )
      {
        previousMillis = currentMillis;
        if(x++==0){
          Serial.print( "Central RSSI: " );
          Serial.println( central.rssi() );
        }

        if( central.rssi() != 0 )
        {
          digitalWrite( RSSI_LED_PIN, LOW );
        
          //if ( IMU.accelerationAvailable() )
          {
            accelerationX +=0.1;
            //IMU.readAcceleration( accelerationX, accelerationY, accelerationZ );
            accelerationCharacteristic.writeValue( accelerationX );
          }

          
          counterCharacteristic.writeValue( counter++ );
          counter+=0x1000;
        }
        else
        {
          digitalWrite( RSSI_LED_PIN, HIGH );
        }
        //counter++;
      } // intervall
    } // while connected

    Serial.print( F( "Disconnected from central: " ) );
    Serial.println( central.address() );
  } // if central
} // loop



bool setupBleMode()
{
  //String aS ="1String";
  uint8_t i=0;
  if ( !BLE.begin() )
  {
    return false;
  }

  // set advertised local name and service UUID:
  BLE.setDeviceName( "Arduino Nano 33 BLE" );
  BLE.setLocalName( "Arduino Nano 33 BLE" );
  BLE.setAdvertisedService( testService );

  // BLE add characteristics
  testService.addCharacteristic( accelXCharacteristic );
  testService.addCharacteristic( accelYCharacteristic );
  testService.addCharacteristic( accelZCharacteristic );
  testService.addCharacteristic( stringCharacteristic );
  testService.addCharacteristic( buffCharacteristic );



  // add service
  BLE.addService( testService );
  for(i=0;i<20;i++) buf[i]=i+1;
  // set the initial value for the characeristic:
  accelerationCharacteristic.writeValue( 0.0 );
  counterCharacteristic.writeValue( 0 );
  buf[10]=0;
  buffCharacteristic.writeValue( buf,20 );
  //buffCharacteristic.
  // start advertising
  BLE.advertise();

  return true;
}
