
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
#include "SAMD_PWM.h"

#define LED_PIN     11
#define LED_COUNT  1
#define BRIGHTNESS 50

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

#define HV_SWITCH_PIN 1
#define NUM_OF_PINS 4
// pin 4: TCC0_CH0, Pin 5:TCC0_CH1, pin 9: TCC1_CH3, pin 10: TCC0_CH2
uint32_t PWM_Pins[]   = { 4, 5, 9, 10 };
//creates pwm instance
SAMD_PWM* PWM_Instance[NUM_OF_PINS];
float frequency[] = { 10000.0f, 10000.0f, 10000.0f, 10000.0f };
float dutyCycle[] = { 10.0f, 10.0f, 10.0f, 10.0f };

#define PWM1 3        // A3
#define PWM2 2        // A2

#define PWM3 10       // A10
#define PWM4 9        // A9

//UART variables
#define PKT_LEN 6

uint8_t integer_array[PKT_LEN];
const byte numChars = 20;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing
boolean newData = false;

void setup() {
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(20); // Set BRIGHTNESS to about 1/5 (max = 255)

  // initialize PWM pins
  //assigns PWM frequency of 1.0 KHz and a duty cycle of 0%
  for (uint8_t index = 0; index < NUM_OF_PINS; index++)
  {
    PWM_Instance[index] = new SAMD_PWM(PWM_Pins[index], frequency[index], dutyCycle[index]);

    if (PWM_Instance[index])
    {
      PWM_Instance[index]->setPWM();
    }
  }
  // initialize digital pin HV_SWITCH_PIN as an output.
  pinMode(HV_SWITCH_PIN, OUTPUT);
  delay(10);

  //enable HW circuit
  digitalWrite(HV_SWITCH_PIN, HIGH);

  //startup swipe
  tube_swipe_startup();
  delay(500);

  //start cooms
  Serial1.begin(460800);
}

void loop() {
  recvWithStartEndMarkers();
  if (newData == true) {
    parseData();
    processNewData();

    Serial1.flush();
    newData = false;
  }
  else
  {
    //comms warning
    turnTubeBoardOFF_blue();
  }
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
    while (Serial1.available() > 0 && newData == false) {
        rc = Serial1.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void parseData() {      // split the data into its parts
  uint8_t index_ptr = 0;
  integer_array[0] = (uint8_t)receivedChars[index_ptr];     // convert this part to an integer
  index_ptr++;
  integer_array[1] = (uint8_t)receivedChars[index_ptr];     // convert this part to an integer
  index_ptr++;
  integer_array[2] = (uint8_t)receivedChars[index_ptr];     // convert this part to an integer
  index_ptr++;
  integer_array[3] = (uint8_t)receivedChars[index_ptr];     // convert this part to an integer
  index_ptr++;
  integer_array[4] = (uint8_t)receivedChars[index_ptr];     // convert this part to an integer
  index_ptr++;
  integer_array[5] = (uint8_t)receivedChars[index_ptr];     // convert this part to an integer
}

void processNewData(){
  boolean checkIntegrity = checkCRC();
  uint8_t Power_status = integer_array[0];
  if(checkIntegrity == true)
  {
    //enable HW circuit
    digitalWrite(HV_SWITCH_PIN, Power_status);
    if(Power_status != 0)
    {
      //set LED to green
      colorWipe(strip.Color(  0, 255,   0)     , 10); // Green
      //treat PWM pins based on UART read
      //analogWrite values from 0 to 255
      analogWrite(PWM1, integer_array[1]);
      analogWrite(PWM2, integer_array[2]);
      analogWrite(PWM3, integer_array[3]);
      analogWrite(PWM4, integer_array[4]);
    }
    else
    {
      turnTubeBoardOFF_red();
    }
  }
  else
  {
    turnTubeBoardOFF_red();
  }
}

void turnTubeBoardOFF_red(){
  //set LED to red
  colorWipe(strip.Color(255,   0,   0)     , 10); // Red 
  //set PWM pins to 0
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
  analogWrite(PWM3, 0);
  analogWrite(PWM4, 0);
  //wait before closing
  delay(10);
  //disable HW circuit
  digitalWrite(HV_SWITCH_PIN, LOW);
}

void turnTubeBoardOFF_blue(){
  //set LED to red
  colorWipe(strip.Color(0,   0,   255)     , 10); // blue 
}

boolean checkCRC(){
  uint8_t calculatedCRC = (integer_array[1] + integer_array[2] + integer_array[3] + integer_array[4])/4;
  boolean retVal = false;

  if(integer_array[PKT_LEN - 1] == calculatedCRC)
  {
    Serial.print(calculatedCRC);
    Serial.print("=");
    Serial.print(integer_array[PKT_LEN - 1]);
    Serial.println();
    retVal = true;
  }
  else
  {
    Serial.print(calculatedCRC);
    Serial.print("=");
    Serial.print(integer_array[PKT_LEN - 1]);
    Serial.println('b');
  }
  return retVal;
}

// Constexpr construction
uint8_t makeByte(uint8_t highNibble, uint8_t lowNibble)
{
    return (((highNibble & 0xF) << 4) | ((lowNibble & 0xF) << 0));
}

// Constexpr high nibble extraction
uint8_t getHighNibble(uint8_t byte)
{
    return ((byte >> 4) & 0xF);
}

// Constexpr low nibble extraction
uint8_t getLowNibble(uint8_t byte)
{
    return ((byte >> 0) & 0xF);
}

void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

void tube_swipe_startup()
{
  // fade in from min to max in increments of 5 points:
  for (int fadeValue = 0; fadeValue <= 255; fadeValue += 5) {
    // sets the value (range from 0 to 255):
    analogWrite(PWM1, fadeValue);
    analogWrite(PWM2, fadeValue);
    analogWrite(PWM3, fadeValue);
    analogWrite(PWM4, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }

  // fade out from max to min in increments of 5 points:
  for (int fadeValue = 255; fadeValue >= 0; fadeValue -= 5) {
    // sets the value (range from 0 to 255):
    analogWrite(PWM1, fadeValue);
    analogWrite(PWM2, fadeValue);
    analogWrite(PWM3, fadeValue);
    analogWrite(PWM4, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }
  digitalWrite(HV_SWITCH_PIN, LOW);
}
