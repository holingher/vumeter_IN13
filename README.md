# SpectrumAnalyzer_SPDIF_IN_Nixie_IN-13
Project started from and inspired from https://github.com/wyager/vumeter which uses Rust language. My goal was to re-implement in Arduino with all hardware changes that were necessary. 
Nixie IN-13: 110 $ on ebay (cost might vary but I bought 10 pieces)
Teensy40: 50 $/pcs
QT PY: 5 $/pcs (found a local knock off)
ORJ-8: 5 $/3 pcs (couldn't find a reliable source so I bought these on aliexpress)

Hardware (most of the components were ordered from https://www.farnell.com/):
- 1x Teensy40: since a exmaple was already in place, I choose to stick with Teensy40. This dev board is used in a lot of audio projects. More information can be found here: https://www.pjrc.com/store/teensy40.html
- 4x QT PY: in the original project, ATSAMD11 was used, but I did not managed to make it work and keep the Rust implementation. Since the flash is very limited to put Arduino BTL on it, I choosed to change it with a QT PY which uses ATSAMD21 with more Flash but with the same processing power. The purpose of this dev board is to receive on UART @460800 baudrate data from Teensy40 and to control the indicated tube/band.
- 1x VN7140: the purpose of this IC is to control the power input for all connected boards using an digital output from Teensy40. The input for this logic is the Pin 15 which is the receiver from SPIDF
- 1x ORJ-8: SPIDF connector
- 1x LM1117-5: power supply for Teensy40
- 4x LM1117-3.3: power supply for QT PY
- 4x NCH8200HV: High Voltage power supply for Nixie IN-13 tubes
- 4x TPS22810DBVR: the purpose of this IC is to control the power input for a specific tube board using: 1) a jumper pin on input line and/or 2) a digital output from QT PY
- 16x Nixie IN-13 tubes (numbers might vary if you want to build an vumeter with 4, 8, 12 or 16. Presented numbers are for 16 bands vumeter)
- 16x MJE340
- Resistors (size: 3216): 550k, 5k, 330, 1k (you need to buy them in bulk so I don't put here a number)
- 16x Potentiometers: 3266X-1-500 (or 50k replacement) -> used to calibrate the tubes
- Capacitors (size: 3216): 10 uF (you need to buy them in bulk so I don't put here a number)

PCB's were inspired from the Will's project and I tried to reproduce them as close as possible. I changed some sizes to fit the QT PY and also on audio board to be easily accesible. The boards were ordered from https://jlcpcb.com/

Software(Arduino based code):
1) Teensy40:
   The goal was to take an audio signal, in this case digital (optical), and process it to control the tubes. I will split this in functionalities:
   a) Audio processing
     Input is an ORJ-8 connector (SPDIF) which has 3 pins: 3v3, GND and OUT. OUT is connected to Pin 15 at Teensy40 according to the specification from https://www.pjrc.com/store/teensy40.html
     Pin 15 is dedicated for SPDIF input. This input is then used by Audio library (https://www.pjrc.com/teensy/td_libs_Audio.html) and configured using the dedicated tool specified here:     
     https://www.pjrc.com/teensy/gui/index.html?info=AudioInputAnalogStereo.
     ![image](https://github.com/holingher/vumeter_IN13/assets/33606845/b26d9750-43c8-47f4-accb-5684b04c48ea)
     You can re-create the path and check the comments for each module and what function it has.
     Main takeaway is that Audio library is capable of CD quality (16 bits, 44.1 kHz)
  
     The input is taken using AsyncAudioInputSPDIF3 and then is band filtered using
     fft1024.read(firstBin, lastBin);
          Read several frequency bins, returning their sum. The higher audio octaves are represented by many bins, which are typically read as a group for audio visualization.
     The output for each read(band) is stored into an array which is then used at the end of current cycle to output the information on UART.
  
     During tests it was observed that while playing a song with 12 bands (12 tubes connected) a processorUsage of maximum 30%. So there is enough headroom for more complicated tasks 
   
   b) Data transfer to tube boards
     For each tube board a serial port from Teensy to output the relevant information. Before the actual output there is some extra processing done to transform the output:
      - from linear to exponential to avoid using only the bottom part of the tubes and to have a bit more visual impact on user. Formula presented in the excel below
  [formula_pow255_0_7.xlsx](https://github.com/holingher/vumeter_IN13/files/14546797/formula_pow255_0_7.xlsx)
      - map was required since on QT PY I want to analogWrite the output - range 0 - 255
      - constrain to avoid overflow during the serial input phase
      In order to avoid issues with interpretation on QT PY, I implementated a simple protocol where '<' is start payload and '>' is end of payload.
      The payload contains: 1 byte with PowerStatus
                            4 bytes for each tube to know the PWM value for each
                            1 byte CRC
        
   c) Power control
    As input I do an average over all frequencies and if the result is 0, then after 20 seconds, the power will be cut for tube boards. This is controlled by the dedicated IC's
    TODO: Check Snooze library to put the Teensy40 into sleep

   d) debug analysis
     There is some implementation which can be activated or is already activated which outputs information over the Teensy40 USB interface (using serial monitor)
3) QT PY
   a) Data reception from audio board
      Each tube board is seen as slave and it can only receive data. The reception is done using the protocol described above. The received data is then passed trough some checks like CRC check or powerStatus check.
   b) tube control
      The value calculated by audio board is directly fed into analogWrite since the processing is already done.
   c) power control
      The first byte from the payload is controlling the power status.
   d) debug analysis
      QT PY is mainly used for RGB projects so it has an RGB LED pre-attached. With this software implementation:
       Green: data received and processed,
       Blue: data is not recieved
       Red: Data is not reliable - CRC failed
     There is some implementation which can be activated or is already activated which outputs information over the QT PY USB interface (using serial monitor)
