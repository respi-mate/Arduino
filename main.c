/**************************************************************************/
/*!
    @file     Adafruit_MMA8451.h
    @author   K. Townsend (Adafruit Industries)
    @license  BSD (see license.txt)

    This is an example for the Adafruit MMA8451 Accel breakout board
    ----> https://www.adafruit.com/products/2019

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <SPI.h>
#include <Filters.h>
#include "Adafruit_BluefruitLE_SPI.h"
#include "BluefruitConfig.h"

#define SAMPLING_PERIOD 200

Adafruit_MMA8451 mma = Adafruit_MMA8451();
sensors_event_t event;

float xacc = 0;
float yacc = 0;
float zacc = 0;

float xcal = 0;
float ycal = 0;
float zcal = 0, ztemp = 0;

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#define FACTORYRESET_ENABLE         0  // change to 0 when deploying project. allows the module to bond with ipad
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

float filterfreq = 1.0;
FilterOnePole lowpassFilter1( LOWPASS, filterfreq );
FilterOnePole lowpassFilter2( LOWPASS, filterfreq );
FilterOnePole lowpassFilter3( LOWPASS, filterfreq );

void setup(void) {
  Serial.begin(115200);
  Serial.setTimeout(50);

  /* Initialise the module */
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

      /* Change the device name to make it easier to find */
  if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=Aishwarya's Bluefruit" )) ) {
    error(F("Could not set device name?"));
  }

  /* Set module to DATA mode */
  ble.setMode(BLUEFRUIT_MODE_DATA);
  
  Serial.println("Adafruit MMA8451 test!");

  if (! mma.begin()) {
    ble.println("Couldnt start");
    while (1);
  }
  ble.println("MMA8451 found!");
  
  mma.setRange(MMA8451_RANGE_2_G);
  mma.setDataRate(MMA8451_DATARATE_800_HZ);
  
  Serial.print("Range = "); Serial.print(2 << mma.getRange());  
  Serial.println("G");

  Serial.println("Initialising...");
  for(int i = 0; i < 200; i++)
  {
    mma.getEvent(&event);
    xcal += event.acceleration.x;
    ycal += event.acceleration.y;
    ztemp += event.acceleration.z;
    delay(20);
  }

  xcal /= 5*SAMPLING_PERIOD;
  ycal /= 5*SAMPLING_PERIOD;
  ztemp /= 5*SAMPLING_PERIOD;
  zcal = 9.81 - ztemp;

  Serial.println("Finished Initialising!");
  Serial.print("X bias: "); Serial.print(xcal);
  Serial.print("\tY bias: "); Serial.print(ycal);
  Serial.print("\tZ bias: "); Serial.print(zcal);
  Serial.println();
}

void loop() {

  mma.getEvent(&event);
  //xacc = lowpassFilter1.input(event.acceleration.x) - xcal;
  //yacc = lowpassFilter2.input(event.acceleration.y) - ycal;
  zacc = lowpassFilter3.input(event.acceleration.z) - zcal;

  //String btooth = String(zacc);
  ble.println(String(zacc));

  delay(SAMPLING_PERIOD);
  
}
