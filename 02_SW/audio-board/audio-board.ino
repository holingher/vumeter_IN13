#undef ARDUINO_TEENSY_MICROMOD

#include <Audio.h>
#include <MsTimer2.h>
//#include "CRC8.h"
//#include "CRC.h"
#include <SoftwareSerial.h> // spelling may be wrong

#define PIN_POWER_SET 6
#define PIN_SLEEP_SET 7

#define PIN_SWITCH_A 9
#define PIN_SWITCH_B 10
#define PIN_SWITCH_C 11
#define PIN_SWITCH_D 12

#define PIN_LED 13

#define PKT_LEN 8

//CRC8 crc(0x1d, 0, 0x70, false, false); // define the CRC parameters.

bool debug_mode_enable = true;
// GUItool: begin automatically generated code
AudioMixer4              mixer1;         //xy=312,134
AudioAnalyzeFFT1024      fft1024;        //xy=467,147
// GUItool: end automatically generated code
AsyncAudioInputSPDIF3     spdifIn(false, false, 100, 20, 80);	//dither = false, noiseshaping = false, anti-aliasing attenuation=100dB, minimum half resampling filter length=20, maximum half resampling filter length=80

AudioConnection          patchCord1(spdifIn, 0, mixer1, 0);
AudioConnection          patchCord2(spdifIn, 1, mixer1, 1);
AudioConnection          patchCord5(mixer1, fft1024);

SoftwareSerial mySerial5(21, 20);//serial5 - first tube_board - pin close to 12V line
SoftwareSerial mySerial2(7, 8);//3rd pin from 12V line
SoftwareSerial mySerial4(16, 17);//pin close to GND


uint8_t Power_status = 0;
// The scale sets how much sound is needed in each frequency range to
// show all 8 bars.  Higher numbers are more sensitive.
//scale is different for different numbers of bands
float offset_scale = 100.0;
// An array to hold the 12 frequency bands
float level[12];
// This array holds the on-screen levels.  When the signal drops quickly,
// these are used to lower the on-screen level 1 bar per update, which
// looks more pleasing to corresponds to human sound perception.
int shown[12];
//determine an average to know when audio is not inputed
int shown_average = 0;
//number of connected tube boards. Default is 0
int tube_board_number = 0;
const int bins = 512;
int bands = 0;

extern float tempmonGetTemp(void);
////////////////////////////////

void setup() {
  /////////////////// AUDIO ///////////////////
  // Audio requires memory to work.
  AudioMemory(12);
  // configure the mixer to equally add left & right
  mixer1.gain(0, 0.5);
  mixer1.gain(1, 0.5);

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

  /////////////////// TIMER CONFIG ///////////////////
  //set timer when audio is not active
  MsTimer2::set(10000, LED_flash_TIMER2); // 10_000 ms period
  delay(100);
  //as init enable power pin
  digitalWrite(PIN_POWER_SET, HIGH);
  digitalWrite(PIN_LED, HIGH);

  delay(1000);
  /////////////////// UART INIT ///////////////////
  //LPUART3 with pins: RX:21 and TX:20     
  mySerial5.begin(460800);  
  //LPUART3 with pins: RX:7 and TX:8     
  mySerial2.begin(460800);

////////////////////////////////////////////  
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

void Probe_SendToTubeBoard(uint8_t tube_board_ID, uint8_t first, uint8_t second, uint8_t third, uint8_t forth){
  uint8_t formatted_pkt[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  formatted_pkt[0] = '<';
  formatted_pkt[1] = Power_status;
  first = map(first, 0, 100, 0, 255);
  formatted_pkt[2] = first;
  second = map(second, 0, 100, 0, 255);
  formatted_pkt[3] = second;
  third = map(third, 0, 100, 0, 255);
  formatted_pkt[4] = third;
  forth = map(forth, 0, 100, 0, 255);
  formatted_pkt[5] = forth;
  formatted_pkt[6] = (first + second + third + forth)/4;
  formatted_pkt[7] = '>';

  switch(tube_board_ID) {
    case 0:
    {
      //Serial.print("calc crc: ");
      //Serial.print(formatted_pkt[6]);
      mySerial5.write(formatted_pkt, PKT_LEN);
      delay(15);
      break;
    }
    case 1:
    {
      //Serial.print("calc crc: ");
      //Serial.print(formatted_pkt[6]);
      mySerial2.write(formatted_pkt, PKT_LEN);
      delay(15);
      break;
    }
    default:
      break;
  }

}

void loop() {
  if(debug_mode_enable){
    tube_board_number = 2;
  }

  if(tube_board_number != 0) {
    ProcessFFT();
  }
  else {
    Serial.println("tube board not sending back the number of bands");
  }
}


void ProcessFFT()
{
  if (fft1024.available()) {
    //we have one board with 4 tubes
    bands = tube_board_number * 4;
    for (int b = 0 ; b < bands ; b++)
    {
      int from = int (exp (log (bins) * b / bands)) ;
      int to   = int (exp (log (bins) * (b+1) / bands)) ;
      if (to > from)
      {
        level[b] = fft1024.read (from, to);

        //Serial.print(from);
        //Serial.print(" to ");
        //Serial.print(to);
        //Serial.print(" ");
        //Serial.print(level[b]);
      }
      //Serial.println(" ");

      // TODO: conversion from FFT data to display bars should be
      // exponentially scaled.  But how keep it a simple example?
 //     int val = level[b] * (offset_scale + tube_board_number*10);
 //     if (val > 100) val = 100;

 //     if (val >= shown[b]) {
        
 //     } else {
 //       if (shown[b] > 0) shown[b] = shown[b] - 1;
 //       val = shown[b];
 //     }
      shown[b] = level[b] * 100;
      Serial.print(shown[b]);
      Serial.print(" ");
      Serial.print(level[b]);
      Serial.print(" ");

      if (b <= bands) {
        shown_average += shown[b];
      } 
      if (b == bands) {
        shown_average = shown_average/bands;
      }

    }
    CheckAudioInput();

    //Serial.print("  average freq: ");
    //Serial.print(shown_average);

    Serial.print("  timer: ");
    Serial.print(MsTimer2::count);

    //Serial.print("  input freq: ");
    //Serial.print(spdifIn.getInputFrequency());

    //Serial.print("  switchVal: ");
    //Serial.print(Switch_readState());

    //Serial.print("  temp: ");
    //Serial.print(tempmonGetTemp());

    //Serial.print("  cpu: ");
    //Serial.print(spdifIn.processorUsage());

//////////////////////// send info to tube boards - start
    //Serial.print("  uart data: ");
    uint8_t boards = 0;
    Probe_SendToTubeBoard(boards, shown[(4 * boards) + 0], shown[(4 * boards) + 1], shown[(4 * boards) + 2], shown[(4 * boards) + 3]);
    
    boards = 1;
    Probe_SendToTubeBoard(boards, shown[(4 * boards) + 0], shown[(4 * boards) + 1], shown[(4 * boards) + 2], shown[(4 * boards) + 3]);
    
    boards = 2;
    Probe_SendToTubeBoard(boards, shown[(4 * boards) + 0], shown[(4 * boards) + 1], shown[(4 * boards) + 2], shown[(4 * boards) + 3]);
    
    //boards = 3;
    //Probe_SendToTubeBoard(boards, shown[(4 * boards) + 0], shown[(4 * boards) + 1], shown[(4 * boards) + 2], shown[(4 * boards) + 3]);
//i    0            1            2
//j    0 1 2 3      4 5 6 7      8 9 10 11
//////////////////////// send info to tube boards - stop

    Serial.println("");
    shown_average = 0;
  }
}

void LED_flash_TIMER2()
{
  //disable power pin
  digitalWrite(PIN_POWER_SET, LOW);
  digitalWrite(PIN_LED, LOW);
  Power_status = 0;
}

uint8_t Switch_readState() {
    uint8_t result = 0;
    for(int i = 0; i < 4; i++) {
        result |= !digitalRead(9+i) << i;
    }
    return result;
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
  }
}
