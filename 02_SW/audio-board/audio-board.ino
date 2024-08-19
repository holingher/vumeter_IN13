#undef ARDUINO_TEENSY_MICROMOD
//#define SNOOZE_T40

#include <Audio.h>
#include <MsTimer2.h>
#include <SoftwareSerial.h> // spelling may be wrong
#include "Hysteresis.h"

#define PIN_POWER_SET 6
#define PIN_SLEEP_SET 7

#define PIN_SWITCH_A 9
#define PIN_SWITCH_B 10
#define PIN_SWITCH_C 11
#define PIN_SWITCH_D 12

#define LEVEL_0 0
#define LEVEL_1 1
#define LEVEL_2 2
#define LEVEL_3 3

#define PIN_LED 13
#define FAN_PIN 23

#define PKT_LEN 8

bool debug_mode_enable = true;
// GUItool: begin automatically generated code
AudioMixer4              mixer1;
AudioAnalyzeFFT1024      fft1024;
// GUItool: end automatically generated code
AsyncAudioInputSPDIF3     spdifIn(true, true, 100, 20, 80);	//dither = false, noiseshaping = false, anti-aliasing attenuation=100dB, minimum half resampling filter length=20, maximum half resampling filter length=80

AudioConnection          patchCord1(spdifIn, 0, mixer1, 0);
AudioConnection          patchCord2(spdifIn, 1, mixer1, 1);
AudioConnection          patchCord5(mixer1, fft1024);

SoftwareSerial mySerial5(21, 20);//serial5 - first tube_board - pin close to 12V line
SoftwareSerial mySerial2(7, 8);//3rd pin from 12V line
SoftwareSerial mySerial6(25, 24);//pin close to GND
SoftwareSerial mySerial1(0, 1);//

uint8_t Power_status = 0;
uint8_t Serial_Delay_ms = 1;
uint8_t Debug_LED_brightness = 5;

// Added samples (and result) will be initialised as uint8_t, hysteresis step 5
Hysteresis <uint8_t> hysteresis(5);

// The scale sets how much sound is needed in each frequency range to
// show all bars.  Higher numbers are more sensitive.
// scale is different for different numbers of bands
float offset_scale = 100.0;

// An array to hold the 12 frequency bands
float level[20];

// This array holds the on-screen levels.  When the signal drops quickly,
// these are used to lower the on-screen level 1 bar per update, which
// looks more pleasing to corresponds to human sound perception.
int shown[20];

//determine an average to know when audio is not inputed
int shown_average = 0;

//number of connected tube boards. Default is 0
uint8_t tube_board_number = 4;

const int bins = 512;
int bands = 0;

extern float tempmonGetTemp(void);
////////////////////////////////
#if defined(__IMXRT1062__)
extern "C" uint32_t set_arm_clock(uint32_t frequency);
#endif

void setup() {
  // underclock Teensy to save a bit of power and cool it down
  // the frames loss will increase on serial
  //Other option is to put a radiator and/or a mini fan
  set_arm_clock(600000000); //temp will reach 80 degree Celsius
  //set_arm_clock(320000000); //temp will reach 67 degree Celsius
  //set_arm_clock(240000000); //temp will reach 64 degree Celsius
  /////////////////// AUDIO ///////////////////
  // Audio requires memory to work.
  AudioMemory(16);
  // configure the mixer to equally add left & right
  mixer1.gain(0, 1);
  mixer1.gain(1, 1);

  /////////////////// TEMP MON ///////////////////
  tempmon_init();

  /////////////////// PIN INIT ///////////////////
  pinMode(PIN_POWER_SET, OUTPUT);
  pinMode(PIN_SLEEP_SET, OUTPUT);

  pinMode(PIN_SWITCH_A, INPUT_PULLUP);
  pinMode(PIN_SWITCH_B, INPUT_PULLUP);
  pinMode(PIN_SWITCH_C, INPUT_PULLUP);
  pinMode(PIN_SWITCH_D, INPUT_PULLUP);

  pinMode(PIN_LED, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);

  /////////////////// TIMER CONFIG ///////////////////
  //set timer when audio is not active
  MsTimer2::set(20000, LED_flash_TIMER2); // 20_000 ms period
  delay(100);
  //as init enable power pin
  digitalWrite(PIN_POWER_SET, HIGH);
  // set the brightness of the debug LED
  analogWrite(PIN_LED, Debug_LED_brightness);

  delay(1000);
  /////////////////// UART INIT ///////////////////
  //LPUART8 or arduino "serial5" with pins: RX:21 and TX:20     
  mySerial5.begin(460800);
  //LPUART4 or arduino "serial2" with pins: RX:7 and TX:8     
  mySerial2.begin(460800);
  //LPUART or arduino "serial1" with pins: RX:0 and TX:1
  mySerial1.begin(460800);
  //LPUART3 or arduino "serial6" with pins: RX:25 and TX:24
  mySerial6.begin(460800);
  ///////////////////////////////////////////
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

void Probe_SendToTubeBoard()
{
  uint8_t formatted_pkt[32] = {0x00};
  const float n = 0.7;
  const uint8_t max_value = 255;
  uint8_t delay_serial = 5;
  uint8_t brightness_level = ReadSwitchState();
  
  if(tube_board_number != 0)
  {
    formatted_pkt[0] = '<';
    formatted_pkt[1] = makeByte(Power_status, brightness_level);
    formatted_pkt[2] = (uint8_t)constrain(pow(map(shown[0], 0, 100, 0, max_value), n)/(pow(max_value, n - 1)), 0, max_value);
    formatted_pkt[3] = (uint8_t)constrain(pow(map(shown[1], 0, 100, 0, max_value), n)/(pow(max_value, n - 1)), 0, max_value);
    formatted_pkt[4] = (uint8_t)constrain(pow(map(shown[2], 0, 100, 0, max_value), n)/(pow(max_value, n - 1)), 0, max_value);
    formatted_pkt[5] = (uint8_t)constrain(pow(map(shown[3], 0, 100, 0, max_value), n)/(pow(max_value, n - 1)), 0, max_value);
    formatted_pkt[6] = (uint8_t)((formatted_pkt[2] + formatted_pkt[3] + formatted_pkt[4] + formatted_pkt[5])/4);
    formatted_pkt[7] = '>';
  }
  
  if((tube_board_number == 2) || (tube_board_number == 3) || (tube_board_number == 4))
  {
    formatted_pkt[8] = '<';
    formatted_pkt[9] = makeByte(Power_status, brightness_level);
    formatted_pkt[10] = (uint8_t)constrain(pow(map(shown[4], 0, 100, 0, max_value), n)/(pow(max_value, n - 1)), 0, max_value);
    formatted_pkt[11] = (uint8_t)constrain(pow(map(shown[5], 0, 100, 0, max_value), n)/(pow(max_value, n - 1)), 0, max_value);
    formatted_pkt[12] = (uint8_t)constrain(pow(map(shown[6], 0, 100, 0, max_value), n)/(pow(max_value, n - 1)), 0, max_value);
    formatted_pkt[13] = (uint8_t)constrain(pow(map(shown[7], 0, 100, 0, max_value), n)/(pow(max_value, n - 1)), 0, max_value);
    formatted_pkt[14] = (uint8_t)((formatted_pkt[10] + formatted_pkt[11] + formatted_pkt[12] + formatted_pkt[13])/4);
    formatted_pkt[15] = '>';
  }

  if((tube_board_number == 3) || (tube_board_number == 4))
  {
    formatted_pkt[16] = '<';
    formatted_pkt[17] = makeByte(Power_status, brightness_level);
    formatted_pkt[18] = (uint8_t)constrain(pow(map(shown[8], 0, 100, 0, max_value), n)/(pow(max_value, n - 1)), 0, max_value);
    formatted_pkt[19] = (uint8_t)constrain(pow(map(shown[9], 0, 100, 0, max_value), n)/(pow(max_value, n - 1)), 0, max_value);
    formatted_pkt[20] = (uint8_t)constrain(pow(map(shown[10], 0, 100, 0, max_value), n)/(pow(max_value, n - 1)), 0, max_value);
    formatted_pkt[21] = (uint8_t)constrain(pow(map(shown[11], 0, 100, 0, max_value), n)/(pow(max_value, n - 1)), 0, max_value);
    formatted_pkt[22] = (uint8_t)((formatted_pkt[18] + formatted_pkt[19] + formatted_pkt[20] + formatted_pkt[21])/4);
    formatted_pkt[23] = '>';
  }
  
  if(tube_board_number == 4)
  {
    formatted_pkt[24] = '<';
    formatted_pkt[25] = makeByte(Power_status, brightness_level);
    formatted_pkt[26] = (uint8_t)constrain(pow(map(shown[12], 0, 100, 0, max_value), n)/(pow(max_value, n - 1)), 0, max_value);
    formatted_pkt[27] = (uint8_t)constrain(pow(map(shown[13], 0, 100, 0, max_value), n)/(pow(max_value, n - 1)), 0, max_value);
    formatted_pkt[28] = (uint8_t)constrain(pow(map(shown[14], 0, 100, 0, max_value), n)/(pow(max_value, n - 1)), 0, max_value);
    formatted_pkt[29] = (uint8_t)constrain(pow(map(shown[15], 0, 100, 0, max_value), n)/(pow(max_value, n - 1)), 0, max_value);
    formatted_pkt[30] = (uint8_t)((formatted_pkt[26] + formatted_pkt[27] + formatted_pkt[28] + formatted_pkt[29])/4);
    formatted_pkt[31] = '>';
  }
  
  if((tube_board_number != 0) && (tube_board_number <= 4))
  {
    mySerial5.write(&formatted_pkt[0], PKT_LEN);
    delay(Serial_Delay_ms);
  }
  
  if((tube_board_number == 2) || (tube_board_number == 3) || (tube_board_number == 4))
  {
    mySerial2.write(&formatted_pkt[8], PKT_LEN);
    delay(Serial_Delay_ms);
  }

  if((tube_board_number == 3) || (tube_board_number == 4))
  {
    mySerial1.write(&formatted_pkt[16], PKT_LEN);
    delay(Serial_Delay_ms);
  }
  
  if(tube_board_number == 4)
  {
    mySerial6.write(&formatted_pkt[24], PKT_LEN);
    delay(Serial_Delay_ms);
  }
}

uint8_t ReadSwitchState()
{
  uint8_t result = 0;
  for(int i = 0; i < 4; i++) {
      result |= !digitalRead(9+i) << i;
  }
  return result;
}

void loop() {
  if(tube_board_number != 0) 
  {
    ProcessFFT();
    Probe_SendToTubeBoard();
  }
  else 
  {
    Serial.println("tube board not sending back the number of bands");
  }
}

void ProcessFFT()
{
  if (fft1024.available()) {
    //we have one board with 4 tubes and we can attach tube_board_number
    bands = tube_board_number * 4;
    Serial.print("   1  ");
    Serial.print("   2  ");
    Serial.print("   3  ");
    Serial.print("   4  ");
    Serial.print("   5  ");
    Serial.print("   6  ");
    Serial.print("   7  ");
    Serial.print("   8  ");
    Serial.print("   9  ");
    Serial.print("  10  ");
    Serial.print("  11  ");
    Serial.print("  12  ");
    Serial.print("  13  ");
    Serial.print("  14  ");
    Serial.print("  15  ");
    Serial.print("  16  ");
    Serial.println("");
    for (int b = 0 ; b < bands ; b++)
    {
      int from = int (exp (log (bins) * b / bands)) ;
      int to   = int (exp (log (bins) * (b+1) / bands)) ;
      if (to >= from)
      {
        level[b] = fft1024.read (from, to);
      }
      
      //Serial.print(from);
      //Serial.print(" to ");
      //Serial.print(to);
      //Serial.print(" ");
      //Serial.print(level[b]);
      //Serial.println(" ");

      shown[b] = level[b] * offset_scale;
      if(shown[b] >= 100) shown[b] = 100;
      if(shown[b] < 10)
      {
        Serial.print("   ");
        Serial.print(shown[b]);
        Serial.print("  ");
      }
      else
      {
        Serial.print("  ");
        Serial.print(shown[b]);
        Serial.print("  ");
      }
      //Serial.print(level[b]);
      //Serial.print(" ");

      if (b <= bands) {
        shown_average += shown[b];
      } 
      if (b == bands) {
        shown_average = shown_average/bands;
      }

    }
    CheckAudioInput();

    Serial.println("");

    Serial.print("F_CPU_ACTUAL=");
    Serial.print(F_CPU_ACTUAL);
    
    double bufferedTime=spdifIn.getBufferedTime();
    Serial.print("  buffered time [micro seconds]: ");
    Serial.print(bufferedTime*1e6,2);

    //Serial.print("  average freq: ");
    //Serial.print(shown_average);

    Serial.print("  timer: ");
    Serial.print(MsTimer2::count);

    Serial.print("  input freq: ");
    Serial.print(spdifIn.getInputFrequency());

    Serial.print("  switchVal: ");
    Serial.print(ReadSwitchState());

    float internal_temp = tempmonGetTemp();
    Serial.print("  temp: ");
    Serial.print(internal_temp);
    internal_temp = hysteresis.add(internal_temp);
    FanControl(internal_temp);
    Serial.print("  temp_h: ");
    Serial.print(internal_temp);

    //Serial.print("  cpu: ");
    //Serial.print(spdifIn.processorUsage());

	  //Serial.print("  Memory usage: ");
    //Serial.print(AudioMemoryUsage());

    //Serial.print("  max number of used blocks: ");
    //Serial.println(AudioMemoryUsageMax()); 

    Serial.println("");
    shown_average = 0;
  }
}

void FanControl(float input_temp)
{
  if(input_temp < 60)
  {
    analogWrite(FAN_PIN, 0);
  }
  else if((input_temp > 60) && (input_temp < 70))
  {
    analogWrite(FAN_PIN, 150);
  }
  else if((input_temp > 70) && (input_temp < 80))
  {
    analogWrite(FAN_PIN, 200);
  }
  else if(input_temp > 80)
  {
    analogWrite(FAN_PIN, 250);
  }
  else
  {
    analogWrite(FAN_PIN, 0);
  }
}

uint8_t counter_general_power = 0;
void LED_flash_TIMER2()
{
  if(counter_general_power == 0)
  {
    //tell slaves to turn off
    Power_status = 0;
    counter_general_power = 1;
  }
  if(counter_general_power == 1)
  {
    //disable overall power
    digitalWrite(PIN_POWER_SET, LOW);
    digitalWrite(PIN_LED, LOW);
    counter_general_power = 0;
  }
}

void CheckAudioInput()
{
  static bool timer_started = false;
  if (shown_average == 0)
  {
    if(timer_started == false)
    {
      MsTimer2::start();
      timer_started = true;
    }
    else //if(timer_started == true)
    {
      //do nothing
    }
  }
  else //if(shown_average != 0)
  {
    if(timer_started == false)
    {
      MsTimer2::stop();
    }
    else //if(timer_started == true)
    {
      MsTimer2::stop();
      timer_started = false;
    }
    //enable power pin
    digitalWrite(PIN_POWER_SET, HIGH);
    digitalWrite(PIN_LED, HIGH);
    Power_status = 1;
    counter_general_power = 0;
  }
}
