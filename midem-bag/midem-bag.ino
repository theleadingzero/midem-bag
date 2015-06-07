#include <SPI.h>
#include "WS2801.h"
#include "Adafruit_BLE_UART.h"

/*****************************************************************************
Hack for Midem Hack Day 2015

by Becky Stewart

*****************************************************************************/

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

// Output pins for WS2801 LEDs
#define DATA 4
#define CLOCK 5
#define NUMLEDS 48
// Set the first variable to the NUMBER of pixels
WS2801 strip = WS2801(NUMLEDS, DATA, CLOCK);

// tassel switches
int bluePin = A0;
int pinkPin = A1;
long lastTasselTime;

// variables for animation
int currFrame = 0;
int currAnimation = 0;
long lastFrameTime;
int rainbowFrame = 0;

/*-----------------------------
  Set up everything
-------------------------------*/
void setup() {
  // turn on pull-up resistor for tassel switch
  pinMode( A0, INPUT_PULLUP );
  pinMode( A1, INPUT_PULLUP );

  // LED set up
  strip.begin();

  // Update LED contents, to start they are all 'off'
  strip.show();

  // start Serial
  Serial.begin(9600);
  //while (!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("MIDEM HACK BAG"));

  // start Bluetooth
  BTLEserial.setDeviceName("BAG"); /* 7 characters max! */
  BTLEserial.begin();

  // start with no connection animation
  lastFrameTime = millis();
  setAnimation(0);

  // tassel debounce
  lastTasselTime = millis();
}

/*-----------------------------
  Continuously loops
-------------------------------*/
void loop() {
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
      Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
      Serial.println(F("* Connected!"));
      setAnimation(1);
    }
    if (status == ACI_EVT_DISCONNECTED) {
      Serial.println(F("* Disconnected or advertising timed out"));
      setAnimation(0);
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status == ACI_EVT_DISCONNECTED) {
    //nextFrame();
  }

  if (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    if (BTLEserial.available()) {
      Serial.print("* "); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));
    }
    // OK while we still have something to read, get a character and print it out
    while (BTLEserial.available()) {
      char c = BTLEserial.read();
      Serial.print(c);

      // red if received a 'r'
      if ( c == 'R' ) {
        //colorWipe(Color(255, 0, 0), 50);
        setAnimation(4);
      }

      // green if received a 'g'
      if ( c == 'G' ) {
        //colorWipe(Color(0, 255, 0), 50);
        setAnimation(5);

        String s = "green";
        // We need to convert the line to bytes, no more than 20 at this time
        uint8_t sendbuffer[20];
        s.getBytes(sendbuffer, 20);
        char sendbuffersize = min(20, s.length());

        Serial.print(F("\n* Sending -> \"")); Serial.print((char *)sendbuffer); Serial.println("\"");

        // write the data
        BTLEserial.write(sendbuffer, sendbuffersize);
      }
    }

    // check if tassel was switched on
    int blueValue = digitalRead( bluePin );
    if ( blueValue == LOW && (millis() - lastTasselTime) > 500) {
      Serial.println("blue tassel");
      // change lights
      colorWipe(Color(0, 0, 255), 50);

      // send message
      String s = "blue tassel";
      // We need to convert the line to bytes, no more than 20 at this time
      uint8_t sendbuffer[20];
      s.getBytes(sendbuffer, 20);
      char sendbuffersize = min(20, s.length());

      Serial.print(F("\n* Sending -> \"")); Serial.print((char *)sendbuffer); Serial.println("\"");

      // write the data
      BTLEserial.write(sendbuffer, sendbuffersize);
      // update debounce time
      lastTasselTime = millis();
    }

    int pinkValue = digitalRead( pinkPin );
    if ( pinkValue == LOW && (millis() - lastTasselTime) > 1000) {
      Serial.println("pink tassel");
      // change lights
      nextAnimation();
      // send message
      String s = "pink tassel";
      // We need to convert the line to bytes, no more than 20 at this time
      uint8_t sendbuffer[20];
      s.getBytes(sendbuffer, 20);
      char sendbuffersize = min(20, s.length());

      Serial.print(F("\n* Sending -> \"")); Serial.print((char *)sendbuffer); Serial.println("\"");

      // write the data
      BTLEserial.write(sendbuffer, sendbuffersize);
      // update debaounce time
      lastTasselTime = millis();
    }
  }
  // write out next frame of animation
  nextFrame();
}

/*-----------------------------
  Set current animation
-------------------------------*/
void setAnimation(int a) {
  currAnimation = a;
  Serial.print("current animation: ");
  Serial.println(currAnimation);
}

/*-----------------------------
  Set next animation
-------------------------------*/
void nextAnimation() {
  currAnimation = (currAnimation + 1) % 3 + 1;
  Serial.print("current animation: ");
  Serial.println(currAnimation);
}

/*-----------------------------
  Display next frame of the current animation
-------------------------------*/
void nextFrame() {
  switch ( currAnimation ) {
    // not connected
    case 0:
      for (int i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, 0);
      }
      strip.setPixelColor(currFrame, Color(200, 0, 100));
      strip.setPixelColor( (currFrame * 2) % strip.numPixels(), Color(200, 0, 100));
      strip.show();   // write all the pixels out
      delay(200);
      break;

    // rainbow
    case 1:
      if ( (millis() - lastFrameTime) > 500 ) {
        for (int j = 0; j < 256; j++) {   // 3 cycles of all 256 colors in the wheel
          for (int i = 0; i < strip.numPixels(); i++) {
            strip.setPixelColor(i, Wheel( (i + j) % 255));
          }
          strip.show();   // write all the pixels out
        }
        lastFrameTime = millis();
      }
      break;

      break;

    // slow rainbow
    case 2:
      if ( (millis() - lastFrameTime) > 4000 ) {
        for (int j = 0; j < 126; j++) {   // 3 cycles of all 256 colors in the wheel
          for (int i = 0; i < strip.numPixels(); i++) {
            strip.setPixelColor(i, Wheel( (i + j) % 255));
          }
          strip.show();   // write all the pixels out
          delay(3);
        }
        lastFrameTime = millis();
      }
      break;

    // solid pink
    case 3:
      for (int i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, Color(100, 0, 100));
      }
      strip.show();
      break;

    //  solid red
    case 4:
      for (int i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, Color(255, 0, 0));
      }
      strip.show();
      break;

    // solid green
    case 5:
      for (int i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, Color(0, 255, 0));
      }
      strip.show();
      break;
  }

  currFrame++; // advance to next frame
  currFrame = currFrame % NUMLEDS; // loop back to 0 when end of array of LEDs
}

/*-----------------------------
-------------------------------*/
void rainbow(uint8_t wait) {
  int i, j;

  for (j = 0; j < 256; j++) {   // 3 cycles of all 256 colors in the wheel
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel( (i + j) % 255));
    }
    strip.show();   // write all the pixels out
    delay(wait);
  }
}

/*-----------------------------
// Slightly different, this one makes the rainbow wheel equally distributed
// along the chain
-------------------------------*/
void rainbowCycle(uint8_t wait) {
  int i, j;

  for (j = 0; j < 256 * 5; j++) {   // 5 cycles of all 25 colors in the wheel
    for (i = 0; i < strip.numPixels(); i++) {
      // tricky math! we use each pixel as a fraction of the full 96-color wheel
      // (thats the i / strip.numPixels() part)
      // Then add in j which makes the colors go around per pixel
      // the % 96 is to make the wheel cycle around
      strip.setPixelColor(i, Wheel( ((i * 256 / strip.numPixels()) + j) % 256) );
    }
    strip.show();   // write all the pixels out
    delay(wait);
  }
}

/*-----------------------------
 fill the dots one after the other with said color
 good for testing purposes
-------------------------------*/
void colorWipe(uint32_t c, uint8_t wait) {
  int i;

  for (i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}


/* Helper functions */

/*-----------------------------
 Create a 24 bit color value from R,G,B
-------------------------------*/
uint32_t Color(byte r, byte g, byte b)
{
  uint32_t c;
  c = r;
  c <<= 8;
  c |= g;
  c <<= 8;
  c |= b;
  return c;
}

/*-----------------------------
  Input a value 0 to 255 to get a color value.
  The colours are a transition r - g -b - back to r
-------------------------------*/
uint32_t Wheel(byte WheelPos)
{
  if (WheelPos < 85) {
    return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
    WheelPos -= 170;
    return Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
