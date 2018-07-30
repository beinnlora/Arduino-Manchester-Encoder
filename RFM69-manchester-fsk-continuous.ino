// Requires  Radiohead library
//http://www.airspayce.com/mikem/arduino/RadioHead/

#include <SPI.h>
#include <RH_RF69.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 433.91

/************ Definitions for SPI pins on common microcontroller boards ******************/
#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
#define RFM69_CS      8
#define RFM69_INT     7
#define RFM69_RST     4
#define LED           13
#endif

#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define LED           13
#endif

#if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
#define RFM69_INT     3  // 
#define RFM69_CS      4  //
#define RFM69_RST     2  // "A"
#define LED           13
#endif

#if defined(ESP8266)    // ESP8266 feather w/wing
#define RFM69_CS      2    // "E"
#define RFM69_IRQ     15   // "B"
#define RFM69_RST     16   // "D"
#define LED           0
#endif

#if defined(ESP32)    // ESP32 feather w/wing
#define RFM69_RST     13   // same as LED
#define RFM69_CS      33   // "B"
#define RFM69_INT     27   // "A"
#define LED           13
#endif

/* Teensy 3.x w/wing
  #define RFM69_RST     9   // "A"
  #define RFM69_CS      10   // "B"
  #define RFM69_IRQ     4    // "C"
  #define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )
*/

/* WICED Feather w/wing
  #define RFM69_RST     PA4     // "A"
  #define RFM69_CS      PB4     // "B"
  #define RFM69_IRQ     PA15    // "C"
  #define RFM69_IRQN    RFM69_IRQ
*/

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

/*###########################################*/
/*
     VARIABLES

  #########################################*/


// Which arduino pin will drive the RFM69 modulation.
// Connect this arduino DIO pin to the RFM69 DIO2 pin (broken out on the Adafruit Feather) for direct pin modulation
int txpin = 9;

//sync bytes (address if you will)
uint8_t sync[2] = {0xAb, 0x56};

//some data to transmit
uint8_t data[20] = {0xAB, 0xb5, 0xaa, 0x55, 0x55, 0xAA, 0xAA, 0xAA, 0xAA};

//manchester pulse durations
uint16_t delay1;
uint16_t delay2;

//capture serial input for experimentation
int incomingByte = 0;

/*===========  SETUP  ===========  */
void setup()
{
  //define our manchester encoding pulse widths. Values below equate to 1200 baud (1/2 chip rate)
  delay1 = 395;
  delay2 = 395;

  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  pinMode(txpin, OUTPUT);



  // manual reset
  digitalWrite(RFM69_RST, LOW);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");

  //configure modem: format is
  /*     typedef struct
       {
           uint8_t    reg_02;   ///< Value for register RH_RF69_REG_02_DATAMODUL
           uint8_t    reg_03;   ///< Value for register RH_RF69_REG_03_BITRATEMSB
           uint8_t    reg_04;   ///< Value for register RH_RF69_REG_04_BITRATELSB
           uint8_t    reg_05;   ///< Value for register RH_RF69_REG_05_FDEVMSB
           uint8_t    reg_06;   ///< Value for register RH_RF69_REG_06_FDEVLSB
           uint8_t    reg_19;   ///< Value for register RH_RF69_REG_19_RXBW
           uint8_t    reg_1a;   ///< Value for register RH_RF69_REG_1A_AFCBW
           uint8_t    reg_37;   ///< Value for register RH_RF69_REG_37_PACKETCONFIG1
       } ModemConfig;

  */
  // bit sync/data mode OFF-continuous, FSK, no mod shaping           = 0x60
  // bit rate (set for 1200bps) - used for recevier (not implemented) = 0x06 0x83
  // deviation ~ 27Khz (needed for my application)                    = 0x01 0xc8
  // rxbw  (not implemented)          ~10khz                          = f4
  // AFCBW
  // packet config: off(continous)
  RH_RF69::ModemConfig t = { 0x60,  0x68, 0x2b, 0x01, 0xC8, 0xf4, 0xf4, 0x00};
  rf69.setModemRegisters( &t );


  /* FREQUENCY*/
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  /*POWER */
  rf69.setTxPower(14, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
  rf69.printRegisters();

}


void loop() {

  // Set mode to Transmit
  rf69.setOpMode(RH_RF69_OPMODE_MODE_TX);
  
  // Send our data - 16x '10' preamble, 2byte sync, then two bytes
  // transmitArray (preamble length, sync bytes[2], length of data to transmit, data[length])
  transmitArray(16, sync, 2, data);
  delay(20);
  
  // put in standby afterwards
  rf69.setOpMode(RH_RF69_OPMODE_MODE_STDBY);

  //for demo, repeat every 2 seconds
  delay(2000);
}

/************** Functions ****************/

void sendZero()
{
  delayMicroseconds(delay1);
  digitalWrite(txpin, HIGH);

  delayMicroseconds(delay2);
  digitalWrite(txpin, LOW);
}

void sendOne()
{
  delayMicroseconds(delay1);
  digitalWrite(txpin, LOW);

  delayMicroseconds(delay2);
  digitalWrite(txpin, HIGH);
}

void transmitArray(uint8_t preamble, uint8_t *syncd, uint8_t numBytes, uint8_t *data)
{
  //send preable
  for ( int8_t i = 0; i < preamble; i++) //send capture pulses
  {
    sendOne(); //end of capture pulses
    sendZero(); //start data pulse
  }
  
  //send sync
  for (uint8_t i = 0; i < 2; i++)
  {
    uint16_t mask = 0x01; //mask to send bits
    uint8_t d = syncd[i]; //^ DECOUPLING_MASK;
    for (uint8_t j = 0; j < 8; j++)
    {
      if ((d & mask) == 0)
        sendZero();
      else
        sendOne();
      mask <<= 1; //get next bit
    }//end of byte
  }//end of data

  //send data
  for (uint8_t i = 0; i < numBytes; i++)
  {
    uint16_t mask = 0x01; //mask to send bits
    uint8_t d = data[i]; //^ DECOUPLING_MASK;
    for (uint8_t j = 1; j <= 8; j++)
    {
      // if (i!=(numBytes-1) && j<=8)
      //  { // last byte

      if ( (i == (numBytes - 1)) && ( j > 6))
      {
        //last byte
      } else
      {
        if ((d & mask) == 0)
          sendZero();
        else
          sendOne();
        mask <<= 1; //get next bit
      }//end of byte
    }

  }//end of data - terminate with zero to be a good neighbour
  sendZero();
}



