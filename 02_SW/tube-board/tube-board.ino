
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define LED_PIN     11
#define LED_COUNT  1
#define BRIGHTNESS 50

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

#define HV_SWITCH_PIN 1
#define NUM_OF_PINS 4

#define PWM1 3        // A3  - PA05 - AIN[5] - TCC0/WO[1] 
#define PWM2 2        // A2  - PA04 - AIN[4] - TCC0/WO[0]

#define PWM3 10       // A10 - PA10 - AIN[18] - TCC1/WO[0] / TCC0/WO[2]
#define PWM4 9        // A9  - PA09 - AIN[17] - TCC0/WO[1] / TCC1/WO[3]

//UART variables
#define PKT_LEN 6

uint8_t integer_array[PKT_LEN];
const byte numChars = 20;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing
boolean newData = false;

uint32_t counter_bad_crc = 0;
uint32_t counter_no_frame_in_time = 0;
uint32_t counter_no_power_status = 0;
uint32_t counter_good_frame = 0;

void setup() {
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(20); // Set BRIGHTNESS to about 1/5 (max = 255)

  // initialize PWM pins
  //I think that at lower PWM speeds and small PWM values, the IN-13 was 
  //showing a spread of current inputs and "smearing" the bargraph output.
  //Raising the PWM speed from the default of 976.56 Hz to the max of 62,500Hz
  // Enable and configure generic clock generator 4
  Setup_PWM_frequency();

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
    counter_no_frame_in_time++;
    turnTubeBoardOFF_blue();
  }
  Serial.print("counter_good_frame: ");
  Serial.print(counter_good_frame);
  Serial.print("      ");
  Serial.print("counter_bad_crc: ");
  Serial.print(counter_bad_crc);
  Serial.print("      ");
  Serial.print("counter_no_frame_in_time: ");
  Serial.print(counter_no_frame_in_time);
  Serial.print("      ");
  Serial.print("counter_no_power_status: ");
  Serial.print(counter_no_power_status);
  Serial.print("      ");
  Serial.println();
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
      counter_good_frame++;
    }
    else
    {
      counter_no_power_status++;
      turnTubeBoardOFF_red();
    }
  }
  else
  {
    counter_bad_crc++;
    turnTubeBoardOFF_red();
  }
}

void turnTubeBoardOFF_red(){
  //set LED to red
  colorWipe(strip.Color(255,   0,   0)     , 10); // Red 
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
   // Serial.print(calculatedCRC);
   // Serial.print("=");
   // Serial.print(integer_array[PKT_LEN - 1]);
   // Serial.println();
    retVal = true;
  }
  else
  {
    //Serial.print(calculatedCRC);
   // Serial.print("=");
   // Serial.print(integer_array[PKT_LEN - 1]);
   // Serial.println('b');
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
    delay(20);
  }

  // fade out from max to min in increments of 5 points:
  for (int fadeValue = 255; fadeValue >= 0; fadeValue -= 5) {
    // sets the value (range from 0 to 255):
    analogWrite(PWM1, fadeValue);
    analogWrite(PWM2, fadeValue);
    analogWrite(PWM3, fadeValue);
    analogWrite(PWM4, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    delay(20);
  }
  digitalWrite(HV_SWITCH_PIN, LOW);
}

#define TCC_CTRLA_PRESCALER_DIV768_Val 768
//set PWM freq to 62,500Hz
// https://www.instructables.com/IN-13-Nixie-Bargraph-Arduino-Control-Circuit/
// https://shawnhymel.com/1710/arduino-zero-samd21-raw-pwm-using-cmsis/
// https://blog.thea.codes/phase-shifted-pwm-on-samd/

// Number to count to with PWM (TOP value). Frequency can be calculated by
// freq = GCLK4_freq / (TCC0_prescaler * (1 + TOP_value))
// With TOP of 47, we get a 1 MHz square wave in this example
uint32_t period = 48 - 1;

void Setup_PWM_frequency()
{
  // Because we are using TCC0, limit period to 24 bits
  period = ( period < 0x00ffffff ) ? period : 0x00ffffff;

  /* Enable the APB clock for TCC0 & TCC1. */
  PM->APBCMASK.reg |= PM_APBCMASK_TCC0 | PM_APBCMASK_TCC1;

  // Enable and configure generic clock generator 4
  GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |          // Improve duty cycle (50 % / 50 %)
                      GCLK_GENCTRL_GENEN |        // Enable generic clock gen
                      GCLK_GENCTRL_SRC_DFLL48M |  // Select 48MHz as source
                      GCLK_GENCTRL_ID(4);         // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Set clock divider of 1 to generic clock generator 4
  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(4) |         // Set clock division (48/4 = 12 MHz)
                     GCLK_GENDIV_ID(4);           // Apply to GCLK4 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  
  // Enable GCLK4 and connect it to TCC0 and TCC1
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |        // Enable generic clock
                      GCLK_CLKCTRL_GEN_GCLK4 |    // Select GCLK4
                      GCLK_CLKCTRL_ID_TCC0_TCC1;  // Feed GCLK4 to TCC0/1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  /* Configure the clock prescaler for each TCC.
    This lets you divide up the clocks frequency to make the TCC count slower
    than the clock. In this case, I'm dividing the 12MHz clock by 256 making the
    TCC operate at 46875Hz. This means each count (or "tick") is 21.3us.
  */
  TCC0->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV256);
  TCC1->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV256);

  /* Use "Normal PWM" */
  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
  /* Wait for bus synchronization */
  while (TCC0->SYNCBUSY.bit.WAVE) {};

  /* Use "Normal PWM" */
  TCC1->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
  /* Wait for bus synchronization */
  while (TCC1->SYNCBUSY.bit.WAVE) {};

  /* Configure the frequency for the PWM by setting the PER register.
   The value of the PER register determines the frequency in the following
   way:

    frequency = GCLK frequency / (TCC prescaler * (1 + PER))

   So in this example frequency = 12Mhz / (256 * (1 + 512)) so the frequency
   is 91,37Hz.
  */
  uint32_t period = 512;

  TCC0->PER.reg = period;
  while (TCC0->SYNCBUSY.bit.PER) {};
  TCC1->PER.reg = period;
  while (TCC1->SYNCBUSY.bit.PER) {};

  /* n for CC[n] is determined by n = x % 4 where x is from WO[x]
   WO[x] comes from the peripheral multiplexer - we'll get to that in a second.
  */
  TCC0->CC[2].reg = period / 2;
  while (TCC0->SYNCBUSY.bit.CC2) {};
  TCC1->CC[1].reg = period / 2;
  while (TCC1->SYNCBUSY.bit.CC2) {};

  /* Configure PA18 and PA07 to be output. */
  PORT->Group[PORTA].DIRSET.reg = PORT_PA04;
  PORT->Group[PORTA].OUTCLR.reg = PORT_PA04;

  PORT->Group[PORTA].DIRSET.reg = PORT_PA05;
  PORT->Group[PORTA].OUTCLR.reg = PORT_PA05;

  PORT->Group[PORTA].DIRSET.reg = PORT_PA10;
  PORT->Group[PORTA].OUTCLR.reg = PORT_PA10;
  
  PORT->Group[PORTA].DIRSET.reg = PORT_PA09;
  PORT->Group[PORTA].OUTCLR.reg = PORT_PA09;

  /* Enable the peripheral multiplexer for the pins. */
  PORT->Group[PORTA].PINCFG[4].reg |= PORT_PINCFG_PMUXEN;
  PORT->Group[PORTA].PINCFG[5].reg |= PORT_PINCFG_PMUXEN;
  PORT->Group[PORTA].PINCFG[9].reg |= PORT_PINCFG_PMUXEN;
  PORT->Group[PORTA].PINCFG[10].reg |= PORT_PINCFG_PMUXEN;

  // Connect TCC0 timer to PA04. Function E is TCC0/WO[0] for PA04.
  // Odd pin num (2*n + 1): use PMUXO
  // Even pin num (2*n): use PMUXE
  PORT->Group[PORTA].PMUX[2].reg = PORT_PMUX_PMUXE_E;
  
  // Connect TCC0 timer to PA05. Function E is TCC0/WO[1] for PA05.
  // Odd pin num (2*n + 1): use PMUXO
  // Even pin num (2*n): use PMUXE
  PORT->Group[PORTA].PMUX[3].reg = PORT_PMUX_PMUXO_E;

  // Connect TCC0 timer to PA10. Function F is TCC0/WO[2] for PA10.
  // Odd pin num (2*n + 1): use PMUXO
  // Even pin num (2*n): use PMUXE
  PORT->Group[PORTA].PMUX[5].reg = PORT_PMUX_PMUXE_F;
  
  // Connect TCC1 timer to PA09. Function F is TCC1/WO[3] for PA09.
  // Odd pin num (2*n + 1): use PMUXO
  // Even pin num (2*n): use PMUXE
  PORT->Group[PORTA].PMUX[5].reg = PORT_PMUX_PMUXO_F;

  TCC0->CTRLA.reg |= (TCC_CTRLA_ENABLE);
  while (TCC0->SYNCBUSY.bit.ENABLE) {};
  TCC1->CTRLA.reg |= (TCC_CTRLA_ENABLE);
  while (TCC1->SYNCBUSY.bit.ENABLE) {};
}