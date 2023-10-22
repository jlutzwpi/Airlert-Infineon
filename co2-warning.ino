/*--- NOTES ---
*/

/*--- INCLUDE LIBRARIES ---*/
// include SPI, MP3 and SD libraries
#include <SPI.h>
#include <Adafruit_VS1053.h>
#include <SD.h>

/*--- INPUT CONSTANTS ---*/
#define BAUD_RATE 115200

/*--- OTHER CONSTANTS ---*/
// These are the pins used for the breakout example
#define BREAKOUT_RESET  9      // VS1053 reset pin (output)
#define BREAKOUT_CS     10     // VS1053 chip select pin (output)
#define BREAKOUT_DCS    8      // VS1053 Data/command select pin (output)
// These are the pins used for the music maker shield
#define SHIELD_RESET  -1      // VS1053 reset pin (unused!)
#define SHIELD_CS     7      // VS1053 chip select pin (output)
#define SHIELD_DCS    6      // VS1053 Data/command select pin (output
//#define echoPin 0 // attach pin D2 Arduino to pin Echo of HC-SR04
//#define trigPin 2 // attach pin D3
#define PIR_MOTION_SENSOR 2
// These are common pins between breakout and shield
#define CARDCS 4     // Card chip select pin
// DREQ should be an Int pin, see http://arduino.cc/en/Reference/attachInterrupt
#define DREQ 3       // VS1053 Data request, ideally an Interrupt pin

/*--- TEST VARIABLES ---*/

/*--- FUNCTION PROTOTYES ---*/

/*--- CLASS INSTANCES ---*/
Adafruit_VS1053_FilePlayer musicPlayer = 
  Adafruit_VS1053_FilePlayer(SHIELD_RESET, SHIELD_CS, SHIELD_DCS, DREQ, CARDCS);
//SR04 sr04 = SR04(echoPin,trigPin);

/*--- MODULE VARIABLES ---*/
//long distance = 0.0;

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.println("Starting up!");
  Serial.println("Witch");
  if (! musicPlayer.begin()) { // initialise the music player
     Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
     while (1);
  }
  Serial.println(F("VS1053 found"));
  
   if (!SD.begin(CARDCS)) {
    Serial.println(F("SD failed, or not present"));
    while (1);  // don't do anything more
  }
   // Set volume for left, right channels. lower numbers == louder volume!
  musicPlayer.setVolume(20,20);
  // If DREQ is on an interrupt pin (on uno, #2 or #3) we can do background
  // audio playing
  musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);  // DREQ int
  
  //set up the ultrasonic sensor
  //pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  //pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  //pir sensor
  pinMode(PIR_MOTION_SENSOR, INPUT);
  
}

void loop() {
  delay(500);
  //if pin is triggered from XMC Boot Kit 1300, play warning message
  if(!digitalRead(PIR_MOTION_SENSOR))
  {
    Serial.println(F("Playing warning!"));
    musicPlayer.playFullFile("warning.wav");
    delay(3000);
  }
}
