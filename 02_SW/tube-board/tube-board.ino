
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define LED_PIN     11
#define LED_COUNT  1
#define BRIGHTNESS 50

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

#define HV_SWITCH_PIN 1
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
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(PWM4, OUTPUT);

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

  if(Power_status != 0)
  {
    if(checkIntegrity == true)
    {
      //set LED to green
      colorWipe(strip.Color(  0, 255,   0)     , 10); // Green
      //enable HW circuit
      digitalWrite(HV_SWITCH_PIN, HIGH);
      //treat PWM pins based on UART read
      //analogWrite values from 0 to 255
      analogWrite(PWM1, map(integer_array[1], 0, 10, 0, 255));
      analogWrite(PWM2, map(integer_array[2], 0, 10, 0, 255));
      analogWrite(PWM3, map(integer_array[3], 0, 10, 0, 255));
      analogWrite(PWM4, map(integer_array[4], 0, 10, 0, 255));
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
  uint8_t calculatedCRC = 0x0;
  boolean retVal = false;
  calculatedCRC = integer_array[0] + integer_array[1] + integer_array[2] + integer_array[3] + integer_array[4];
  if(integer_array[PKT_LEN - 1] == calculatedCRC)
  {
    retVal = true;
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
  //for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(1, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  //}
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
