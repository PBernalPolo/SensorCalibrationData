/*
  GNSS data logger using an Arduino MEGA2560 and 3 NEO-6M GPS.

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
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");

  // If we reached this point, turn the LED off to indicate the user that the setup function is complete, and we are going into the loop.
  digitalWrite( LED_BUILTIN , LOW );
}


void loop()
{
  // Write data of each serial to a file.
  writeGnssDataToSdCard( Serial1 , "datalog1.txt" );
  writeGnssDataToSdCard( Serial2 , "datalog2.txt" );
  writeGnssDataToSdCard( Serial3 , "datalog3.txt" );

}


/**
 * Writes GNSS data received by serialPort to a file.
 */
void writeGnssDataToSdCard( Stream& serialPort , String filename ) {
  // If there is available data,
  if( serialPort.available() ) {
    // try to open the file.
    File dataFile = SD.open( filename , FILE_WRITE );
    // Return if we fail to open the file.
    if( !dataFile ) {
      Serial.println( "Failed to open" + filename );
      return;
    }
    // Send a message through the serial port indicating that we are writting to file.
    Serial.print( "Writting to " + filename + "... " );
    // Write available data to file.
    while( serialPort.available() ) {
      dataFile.print( char(serialPort.read()) );
    }
    // Finally, close the file,
    dataFile.close();
    // and send a message through the serial port saying that we wrote the data.
    Serial.println( "done." );
  }
}
