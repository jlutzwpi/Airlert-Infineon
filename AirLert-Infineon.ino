/*!
 Air-Lert: Audio and text message notification of unsafe environmental conditions when people
 are detected in the room.
 Hardware includes:
 1. Infineon XMC 1300 Boot Kit MCU board
 2. Infineon S2GO RADAR BGT60LTR11 - connected to pins TD (2.6 - pin 2) and PD (2.7 - pin 3)
 3. Pimoroni SGP30 (detects TVOC and eCO2) - connected to Blues Notecarrier via Quiic connector
 4. Blues Notecarrier B (with WiFi Notecard) - connected to XMC 1300 board via I2C pins (SDA P2.1, SCL P2.0)
 5. Arduino Uno R3 with Adafruit Music Maker MP3 shield - connected to pin 2 on Arduino and pin 0.11 (17) on XMC 1300
 6. SD card inserted into MP3 shield with .wav file of warning
 7. Custom 3D printed enclosure
 */

#include <Arduino.h>
/* Include library main header */
#include <bgt60-ino.hpp>
/* Include Arduino platform header */
#include <bgt60-platf-ino.hpp>
#include "SparkFun_SGP30_Arduino_Library.h"
#include <Wire.h>
#include <Notecard.h>

//project ID to connect to Blues Notehub
#define PRODUCT_UID "xxxxxxx"

//WiFi SSID and password, and phone number that is going to receive the text message (make sure you follow international format - USA is +1)
String ssid="xxxxx";
String passwd="xxxxxxx";
String phone = "+1xxxxxxxxxx";

/*
* In case no supported platform is defined, the
* PD and TD pin will be set to the values below.
*/
#ifndef TD
#define TD  2
#endif

#ifndef PD
#define PD  3
#endif

//pin number to trigger playing of wav file on Arduino Uno with MP3 shield
#define BUZZER_PIN 17

/* Create radar object with following arguments:
 *  TD : Target Detect Pin
 *  PD : Phase Detect Pin */
Bgt60Ino radarShield(TD, PD);
SGP30 mySensor; //create an object of the SGP30 class (I2C)
//define notecard
#define myProductID PRODUCT_UID
Notecard notecard;

/* Begin setup function - takes care of initializations and executes only once post reset */
void setup()
{
    /* Set the baud rate for sending messages to the serial monitor */
    Serial.begin(115200);

    // Configures the GPIO pins to input mode
    Error_t init_status = radarShield.init();
    delay(9000);
    /* Check if the initialization was successful */
    if (OK != init_status) {
        Serial.println("Radar Init failed.");
    }
    else {
        Serial.println("Radar Init successful.");
    }
    
    Wire.begin();
    //start notecard on I2C
    notecard.begin();
    //setup notecard parameters
    J *req = notecard.newRequest("hub.set");
    JAddStringToObject(req, "product", myProductID);
    JAddStringToObject(req, "mode", "continuous");
    notecard.sendRequest(req);
    //set it up as a wifi notecard
    req = notecard.newRequest("card.wifi");
    JAddStringToObject(req, "ssid", ssid.c_str());
    JAddStringToObject(req, "password", passwd.c_str());
    notecard.sendRequest(req);
    
    //initialize SGP30 sensor
    if (mySensor.begin() == false) {
      Serial.println("No SGP30 Detected. Check connections.");
      while (1);
    }
    //Initializes sensor for air quality readings
    //measureAirQuality should be called in one second increments after a call to initAirQuality
    mySensor.initAirQuality();

    //initialize pin to be triggered to play audio file
    pinMode(BUZZER_PIN, OUTPUT);
    
    
}

/* Begin loop function - this part of code is executed continuously until external termination */
void loop()
{
    /* Initialize the variable to NO_MOTION to be able to record new events */
    Bgt60::Motion_t motion = Bgt60::NO_MOTION;

   /* The getMotion() API does two things:
        1. Returns the success or failure to detect moving object as a message of type Error_t.
           Any value other than OK indicates failure
        2. Sets recent event in "motion" variable. Events can be: NO_MOTION or MOTION */
    Error_t err = radarShield.getMotion(motion);

    /* Check if API execution is successful */
    if(err == OK)
    {
        /* Cases based on value set in motion variable */
        switch (motion)
        {
            /* Variable "motion" is set to MOTION when moving target is detected */
            case Bgt60::MOTION:
                Serial.println("Target in motion detected!");
                break;
            /*  Variable "motion" is set to NO_MOTION when moving target is not present */
            case Bgt60::NO_MOTION:
                Serial.println("No target in motion detected.");
                break;
        }
    }
    /*  API execution returned error */
    else {
        Serial.println("Error occurred!");
    }

    Bgt60::Direction_t direction = Bgt60::NO_DIR;
    err = radarShield.getDirection(direction);
    /* Check if API execution is successful */
    if(err == OK)
    {
        /* Cases based on value set in motion variable */
        switch (direction)
        {
            /* Variable "motion" is set to MOTION when moving target is detected */
            case Bgt60::APPROACHING:
                Serial.println("Target in APPROACHING!");
                break;
            case Bgt60::DEPARTING:
                Serial.println("Target in DEPARTING!");
                break;
            /*  Variable "motion" is set to NO_MOTION when moving target is not present */
            case Bgt60::NO_DIR:
                Serial.println("No target DIRECTION.");
                break;
        }
    }
    /*  API execution returned error */
    else {
        Serial.println("Direction Error occurred!");
    }

    //get CO2 and TVOC readings
    mySensor.measureAirQuality();
    Serial.print("CO2: ");
    Serial.print(mySensor.CO2);
    Serial.print(" ppm\tTVOC: ");
    Serial.print(mySensor.TVOC);
    Serial.println(" ppb");

    //if CO2 is elevated and motion is detected, activate buzzer and send text message
    if(mySensor.CO2 > 1000 && motion == Bgt60::MOTION)
    {
      Serial.println("Activate warning!");
      digitalWrite(BUZZER_PIN, LOW);
      //send a note to notehub to send a text message
      J *req = notecard.newRequest("note.add");
      if (req != NULL) {
        JAddStringToObject(req, "file", "twilio.qo");
        JAddBoolToObject(req, "sync", true);
        J *body = JCreateObject();
        if (body != NULL) {
          String msg = "Unsafe conditions detected with personnel present! CO2 level: " + 
            String(mySensor.CO2) + " ppm. Evacuate the room!";
          JAddStringToObject(body, "customMessage", msg.c_str());
          JAddStringToObject(body, "customTo", phone.c_str());
          JAddItemToObject(req, "body", body);
        }
        notecard.sendRequest(req);
      }
    }
    else
    {
      Serial.println("Warning off");
      digitalWrite(BUZZER_PIN, HIGH);
    }

    /* Reducing the frequency of the measurements */
    delay(1000);
}
