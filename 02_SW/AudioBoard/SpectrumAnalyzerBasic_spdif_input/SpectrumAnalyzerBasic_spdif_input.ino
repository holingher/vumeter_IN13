#include <Audio.h>
#include <MsTimer2.h>

#define PIN_POWER_SET 6
#define PIN_SLEEP_SET 7

#define PIN_SWITCH_A 9
#define PIN_SWITCH_B 10
#define PIN_SWITCH_C 11
#define PIN_SWITCH_D 12

#define PIN_LED 13

typedef union {
  struct {
    uint32_t board_nb   : 4; // 0..3   [4 bits]
    uint32_t brigthness : 4; // 4..7   [4 bits]
    uint32_t tube1      : 4; // 8..11  [4 bits]  
    uint32_t tube2      : 4; // 12..15 [4 bits]  
    uint32_t tube3      : 4; // 16..19 [4 bits]
    uint32_t tube4      : 4; // 20..23 [4 bits]
    uint32_t crc        : 8; // 24..31 [8 bits]
  } as_nibbles;
  uint8_t as_bytes[3];
} UART_message_u;

bool debug_mode_enable = true;
// GUItool: begin automatically generated code
AudioMixer4              mixer1;         //xy=312,134
AudioAnalyzeFFT1024      fft1024;        //xy=467,147
// GUItool: end automatically generated code
AsyncAudioInputSPDIF3     spdifIn(false, false, 100, 20, 80);	//dither = false, noiseshaping = false, anti-aliasing attenuation=100dB, minimum half resampling filter length=20, maximum half resampling filter length=80

AudioConnection          patchCord1(spdifIn, 0, mixer1, 0);
AudioConnection          patchCord2(spdifIn, 1, mixer1, 1);
AudioConnection          patchCord5(mixer1, fft1024);

// The scale sets how much sound is needed in each frequency range to
// show all 8 bars.  Higher numbers are more sensitive.
//scale is different for different numbers of bands
float offset_scale = 20.0;
// An array to hold the 16 frequency bands
float level[16];
// This array holds the on-screen levels.  When the signal drops quickly,
// these are used to lower the on-screen level 1 bar per update, which
// looks more pleasing to corresponds to human sound perception.
int shown[16];
//determine an average to know when audio is not inputed
int shown_average = 0;
//number of connected tube boards. Default is 0
int tube_board_number = 0;
const int bins = 512;
int bands = 0;

extern float tempmonGetTemp(void);

void setup() {
  //LPUART3 with pins: RX:16 and TX:17
	Serial4.begin(460800);
  // Audio requires memory to work.
  AudioMemory(12);

  while (!Serial4 && (millis() <= 6000));  // Wait for Serial interface     //wait for data available
  String readSerial_string = Serial4.readString();  //read until timeout
  readSerial_string.trim();                        // remove any \r \n whitespace at the end of the String
  //check for number of tube boards and choose the FFT config accordingly
  if (readSerial_string == "nb_1") {
    tube_board_number = 1;
  } else if (readSerial_string == "nb_2") {
    tube_board_number = 2;
  } else if (readSerial_string == "nb_3") {
    tube_board_number = 3;
  } else if (readSerial_string == "nb_4") {
    tube_board_number = 4;
  }
  else {
    tube_board_number = 0;
  }
  Serial.print(" from slaves: ");
  Serial.print(tube_board_number);
  // configure the mixer to equally add left & right
  mixer1.gain(0, 0.5);
  mixer1.gain(1, 0.5);

  tempmon_init();

  pinMode(PIN_POWER_SET, OUTPUT);
  pinMode(PIN_SLEEP_SET, OUTPUT);

  pinMode(PIN_SWITCH_A, INPUT_PULLUP);
  pinMode(PIN_SWITCH_B, INPUT_PULLUP);
  pinMode(PIN_SWITCH_C, INPUT_PULLUP);
  pinMode(PIN_SWITCH_D, INPUT_PULLUP);

  pinMode(PIN_LED, OUTPUT);
  //set timer when audio is not active
  MsTimer2::set(5000, LED_flash_TIMER2); // 5_000 ms period
  
  delay(5000);
  //as init enable power pin
  digitalWrite(PIN_POWER_SET, HIGH);
  digitalWrite(PIN_LED, HIGH);
}

/*
byte Compute_CRC8(byte *bytes, int len) {
  const byte generator = B00101111;   // polynomial = x^8 + x^5 + x^3 + x^2 + x + 1 (ignore MSB which is always 1)
  byte crc = 0;

  while (len--)
  {
    crc ^= *bytes++; // XOR-in the next input byte 

    for (int i = 0; i < 8; i++)
    {
      if ((crc & 0x80) != 0)
      {
        crc = (byte)((crc << 1) ^ generator);
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}
*/

void SendToTubeBoard(int board_nb, int tube1, int tube2, int tube3, int tube4) {
  UART_message_u message;
  message.as_nibbles.board_nb = board_nb;
  message.as_nibbles.brigthness = Switch_readState();
  message.as_nibbles.tube1 = tube1;
  message.as_nibbles.tube2 = tube2;
  message.as_nibbles.tube3 = tube3;
  message.as_nibbles.tube4 = tube4;

  message.as_nibbles.crc = message.as_nibbles.board_nb +
                           message.as_nibbles.brigthness + 
                           message.as_nibbles.tube1 +
                           message.as_nibbles.tube2 +
                           message.as_nibbles.tube3 +
                           message.as_nibbles.tube4;
  Serial4.write((char*)&message, sizeof(message));
/*  
  Serial.print(" ");
  Serial.print(message.as_nibbles.board_nb);
  Serial.print(" ");
  Serial.print(message.as_nibbles.brigthness);
  Serial.print(" ");
  Serial.print(message.as_nibbles.tube1);
  Serial.print(" ");
  Serial.print(message.as_nibbles.tube2);
  Serial.print(" ");
  Serial.print(message.as_nibbles.tube3);
  Serial.print(" ");
  Serial.print(message.as_nibbles.tube4);
  Serial.print(" ");
  Serial.print(message.as_nibbles.crc, HEX);
  Serial.print(" ");
  */
}

void loop() {
  if(debug_mode_enable){
    tube_board_number = 4;
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
        //Serial.print(" ");
      }

      // TODO: conversion from FFT data to display bars should be
      // exponentially scaled.  But how keep it a simple example?
      int val = level[b] * (offset_scale + tube_board_number*10);
      if (val > 8) val = 8;

      if (val >= shown[b]) {
        shown[b] = val;
      } else {
        if (shown[b] > 0) shown[b] = shown[b] - 1;
        val = shown[b];
      }
      Serial.print(shown[b]);
      Serial.print(" ");

      if (b <= bands) {
        shown_average += shown[b];
      } 
      if (b == bands) {
        shown_average = shown_average/bands;
      }
      
    }

    if (tube_board_number == 1) {
      SendToTubeBoard(tube_board_number, shown[0], shown[1], shown[2], shown[3]);
    } else if (tube_board_number == 2) {
      SendToTubeBoard(tube_board_number - 1, shown[0], shown[1], shown[2], shown[3]);
      SendToTubeBoard(tube_board_number , shown[4], shown[5], shown[6], shown[7]);
    } else if (tube_board_number == 3) {
      SendToTubeBoard(tube_board_number - 2, shown[0], shown[1], shown[2], shown[3]);
      SendToTubeBoard(tube_board_number - 1, shown[4], shown[5], shown[6], shown[7]);
      SendToTubeBoard(tube_board_number , shown[8], shown[9], shown[10], shown[11]);
    } else if (tube_board_number == 4) {
      SendToTubeBoard(tube_board_number - 3, shown[0], shown[1], shown[2], shown[3]);
      SendToTubeBoard(tube_board_number - 2, shown[4], shown[5], shown[6], shown[7]);
      SendToTubeBoard(tube_board_number - 1, shown[8], shown[9], shown[10], shown[11]);
      SendToTubeBoard(tube_board_number , shown[12], shown[13], shown[14], shown[15]);
    }
    CheckAudioInput();
    
    Serial.print("  average freq: ");
    Serial.print(shown_average);

    Serial.print("  timer: ");
    Serial.print(MsTimer2::count);
    shown_average = 0;
    //Serial.print("  input freq: ");
    //double inputFrequency=spdifIn.getInputFrequency();
    //Serial.print(inputFrequency);

    uint8_t switch_val = Switch_readState();
    Serial.print("  switchVal: ");
    Serial.print(switch_val);

    double internal_temp = tempmonGetTemp();
    Serial.print("  temp: ");
    Serial.print(internal_temp);

    double pUsageIn=spdifIn.processorUsage(); 
    Serial.print("  cpu: ");
    Serial.println(pUsageIn);
  }
}

void LED_flash_TIMER2()
{
  //disable power pin
  digitalWrite(PIN_POWER_SET, LOW);
  digitalWrite(PIN_LED, LOW);
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
  }
}
