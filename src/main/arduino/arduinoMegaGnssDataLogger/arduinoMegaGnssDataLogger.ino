/*
  GNSS data logger using an Arduino MEGA2560 and 3 NEO-6M GPS.

  This sketch only worked after increasing the buffer size in:
  /home/<user>/.arduino15/packages/arduino/hardware/avr/1.8.6/cores/arduino/HardwareSerial.h
  by modifying the line:
  #define SERIAL_RX_BUFFER_SIZE 64
  to
  #define SERIAL_RX_BUFFER_SIZE 1024 //64

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


////////////////////////////////////////////////////////////////
// PARAMETERS
////////////////////////////////////////////////////////////////
#define CHIP_SELECT 53  // for MEGA2560
#define BUFFER_SIZE 1024  // This value must be greater or equal to SERIAL_RX_BUFFER_SIZE (see sketch description on top)


////////////////////////////////////////////////////////////////
// VARIABLES
////////////////////////////////////////////////////////////////
uint8_t buffer[BUFFER_SIZE];



////////////////////////////////////////////////////////////////
// MAIN CODE
////////////////////////////////////////////////////////////////

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
  if( !SD.begin( SPI_FULL_SPEED , CHIP_SELECT ) ) {
    Serial.println( " card failed, or not present." );
    // don't do anything more:
    while (1);
  }
  Serial.println(" card initialized.");

  // LED off when setup is complete, and we are going into the loop.
  digitalWrite( LED_BUILTIN , LOW );
}


void loop()
{
  // Continously copy data from serial to the SD card.
  writeSerialToSdCard( Serial1 , "datalog1.txt" );
  writeSerialToSdCard( Serial2 , "datalog2.txt" );
  writeSerialToSdCard( Serial3 , "datalog3.txt" );
}



////////////////////////////////////////////////////////////////
// FUNCTIONS
////////////////////////////////////////////////////////////////

/**
 * Writes data received through a serial port to a file in the SD card.
 */
void writeSerialToSdCard( Stream& serialPort , String filename )
{
  // Only if there is something to write,
  if( serialPort.available() > 0 ) {
    // Open the data file.
    File dataFile = SD.open( filename , O_WRITE | O_CREAT | O_APPEND );
    // If we fail th open the file, give out info and return.
    if( !dataFile ) {
      if( Serial ) {
        Serial.write( "Error opening " );
        Serial.print( filename );
        Serial.write( "\n" );
      }
      return;
    }
    // If we got here, we managed to open the file and can write data.
    if( Serial ) {
      Serial.write( "Writting to " );
      Serial.print( filename );
      Serial.write( "..." );
    }
    // Copy available serial port bytes into the buffer,
    int numberOfBytesRead = serialPort.readBytes( buffer , serialPort.available() );
    // and write the copied content into the SD card.
    dataFile.write( buffer , numberOfBytesRead );
    // Finally close the file and print to let the user know that we wrote into the file.
    dataFile.close();
    if( Serial ) {
      Serial.write( " done.\n" );
    }
  }
}
