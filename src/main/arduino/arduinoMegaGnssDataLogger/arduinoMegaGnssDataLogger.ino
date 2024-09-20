/*
  GNSS data logger using an Arduino MEGA2560 and 3 NEO-6M GPS.

  This sketch only worked after increasing the buffer size in:
  /home/<user>/.arduino15/packages/arduino/hardware/avr/1.8.6/cores/arduino/HardwareSerial.h
  by modifying the line:
  #define SERIAL_RX_BUFFER_SIZE 64
  to
  #define SERIAL_RX_BUFFER_SIZE 512 //64

  Connections:
    SD card attached to SPI bus as follows:
    * GND - GND
    * 3V3 - 3.3V
    * CS - pin 53
    * MOSI - pin 51
    * SCK - pin 52
    * MISO - pin 50

  After connection, check that everything is working properly by looking at the LED light:
  - On indicates doing setup.
  - Off indicates going into loop; if it never goes off, then something is not working properly; check SD card or press the RESET button.
*/


#include <SPI.h>
#include <SD.h>


const int chipSelect = 53;  // for MEGA2560
const int bufferSize = 1024;
#define SERIAL_AVAILABLE_THRESHOLD 16

uint8_t buffer1[bufferSize];
uint8_t buffer2[bufferSize];
uint8_t buffer3[bufferSize];

int bufferIndex1;
int bufferIndex2;
int bufferIndex3;


void setup()
{
  // Set the led pin as output.
  pinMode(LED_BUILTIN, OUTPUT);
  // Turn the LED on to help the user see that we are in the setup function.
  digitalWrite( LED_BUILTIN , HIGH );

  // Initialize serial ports.
  Serial.begin(9600);
  // Next serial ports will read from GNSS serial.
  Serial1.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);
  // Wait a bit to check if something is being received.
  delay( 1100 );
  // Check if all GNSS are sending info.
  if( Serial1.available() ) {
    Serial.println( "Serial1 up." );
  }
  if( Serial2.available() ) {
    Serial.println( "Serial2 up." );
  }
  if( Serial3.available() ) {
    Serial.println( "Serial3 up." );
  }

  // Initialize SD card.
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if( !SD.begin( SPI_FULL_SPEED , chipSelect ) ) {
    Serial.println( " card failed, or not present." );
    // don't do anything more:
    while (1);
  }
  Serial.println(" card initialized.");

  bufferIndex1 = 0;
  bufferIndex2 = 0;
  bufferIndex3 = 0;

  // LED off when setup is complete, and we are going into the loop.
  digitalWrite( LED_BUILTIN , LOW );
}


void loop()
{
  if( Serial1.available() > 0 ) {
    collectGnssData( Serial1 , buffer1 , bufferIndex1 );
  } else if( Serial2.available() > 0 ) {
    collectGnssData( Serial2 , buffer2 , bufferIndex2 );
  } else if( Serial3.available() > 0 ) {
    collectGnssData( Serial3 , buffer3 , bufferIndex3 );
  } else {
    writeBufferToSd( buffer1 , bufferIndex1 , "datalog1.txt" );
    writeBufferToSd( buffer2 , bufferIndex2 , "datalog2.txt" );
    writeBufferToSd( buffer3 , bufferIndex3 , "datalog3.txt" );
  }
}


void collectGnssData( Stream& serialPort , uint8_t* buffer , int& bufferIndex )
{
  buffer[bufferIndex++] = serialPort.read();
  //int numberOfBytesRead = serialPort.readBytes( &buffer[bufferIndex] , serialPort.available() );
  //bufferIndex += numberOfBytesRead;
}


void writeBufferToSd( uint8_t* buffer , int& bufferIndex , String filename )
{
  if( bufferIndex > 0 ) {
    File dataFile = SD.open( filename , O_WRITE | O_CREAT | O_APPEND );
    if( dataFile ) {
      if( Serial ) {
        Serial.write( "Writting to " );
        Serial.print( filename );
        Serial.write( "..." );
      }
      dataFile.write( buffer , bufferIndex );
      dataFile.close();
      bufferIndex = 0;
      if( Serial ) {
        Serial.write( " done.\n" );
      }
    } else {
      if( Serial ) {
        Serial.write( "Error opening " );
        Serial.print( filename );
        Serial.write( "\n" );
      }
    }
  }
}
