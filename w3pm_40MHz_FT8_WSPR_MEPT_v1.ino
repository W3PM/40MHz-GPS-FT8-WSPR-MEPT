
/*
  File: w3pm_8_meter_beacon

  Si5351 bandswitched WSPR and FT8 source using GPS timing to use as an 8 meter
  manned experimental transmitter or beacon.

  Permission is granted to use, copy, modify, and distribute this software
  and documentation for non-commercial purposes.

  Copyright (C) 2022,  Gene Marcus W3PM GM4YRE

  11 July, 2022

  The code was sucessfully compiled using Arduino 1.8.13

   NOTE! Uses library SSD1306Ascii by Bill Greiman
        Load from: Sketch > Include Library > Manage Libraries

  ------------------------------------------------------------------------
  Nano Digital Pin Allocations follow:
  ------------------------------------------------------------------------
  D0/RX to GPS TX (this pin may differ on the Nano board)
  D1
  D2  GPS 1pps input
  D3  pushButton - controls transmitter on/off
  D4
  D5  2.5 MHz input from Si5351 CLK0 pin
  D6
  D7
  D8
  D9
  D10
  D11
  D12
  D13

  A0/D14 WSPR - XmitLED
  A1/D15 FT8 - XmitLED
  A2/D16 Transmitter ON/OFF LED
  A3/D17
  A4/D18 Si5351 & OLED SDA
  A5/D19 Si5351 & OLED SCL

*/


// Enter desired WSPR transmit offset from dial frequency below:
// Transmit offset frequency in Hz. Range = 1400-1600 Hz (used to determine TX frequency within WSPR window)
unsigned long TXoffset = 1522UL;

// Set to true if you want the WSPR frequency to change within the 200 Hz WSPR freuency window
bool FreqHopTX = false;

// Enter desired FT8 transmit offset from dial frequency below:
// Transmit offset frequency in Hz. Range = 200-2500 Hz (used to determine FT8 TX frequency within FT8 window)
unsigned long FT8_TXoffset = 1230UL;

// Set to true if you want the FT8 frequency to change within the 2500 Hz FT8 freuency window
bool FT8_FreqHopTX = false;



/*
  ----------------------------------------------------------------------------------------------------
                       Scheduling configuration follows:
  ----------------------------------------------------------------------------------------------------
  Format: (TIME, MODE, BAND),

  TIME: WSPR is in 2 minute increments past the top of the hour
        FT8 is in 1 minute increments past the top of the hour

  MODE: 1-WSPR 2-FT8

  BAND: is 'TXdialFreq' band number beginng with 0  i.e. 0 = 40620000 kHz

  Note: Ensure format is correct
        i.e. brackets TIME comma MODE comma BAND comma brackets comma {24,1,11},

  example follows:

  const int schedule [][3] = {
  {0, 1, 0},   // WSPR at top of the hour on 40620.000 kHz
  {2, 2, 1},   // FT8 at 2 minutes past the hour on 40680.000 kHz
  {3, 2, 1},   // FT8 at 3 minutes past the hour on 40680.000 kHz
  {20, 1, 1},  // WSPR at 20 minutes past the hour on 40680.000 kHz
  {58, 1, 1},  // WSPR at 58 minutes past the hour on 40680.000 kHz
  (-1,-1,-1),  // End of schedule flag
  };
  ----------------------------------------------------------------------------------------------------
*/

const int schedule [][3] = {
  {0, 1, 1},
  {2, 1, 1},
  {4, 2, 1},
  {5, 2, 1},
  {6, 2, 1},
  {7, 2, 1},
  {8, 1, 0},
  {10, 1, 1},
  {12, 1, 1},
  {14, 2, 1},
  {15, 2, 1},
  {16, 2, 1},
  {17, 2, 1},
  {18, 1, 0},
  {20, 1, 1},
  {22, 1, 1},
  {24, 2, 1},
  {25, 2, 1},
  {26, 2, 1},
  {27, 2, 1},
  {28, 1, 0},
  {30, 1, 1},
  {12, 1, 1},
  {14, 2, 1},
  {15, 2, 1},
  {16, 2, 1},
  {17, 2, 1},
  {18, 1, 0},
  {20, 1, 1},
  {22, 1, 1},
  {24, 2, 1},
  {25, 2, 1},
  {26, 2, 1},
  {27, 2, 1},
  {28, 1, 0},
  {30, 1, 1},
  {32, 1, 1},
  {34, 2, 1},
  {35, 2, 1},
  {36, 2, 1},
  {37, 2, 1},
  {38, 1, 0},
  {40, 1, 1},
  {42, 1, 1},
  {44, 2, 1},
  {45, 2, 1},
  {46, 2, 1},
  {47, 2, 1},
  {48, 1, 0},
  {50, 1, 1},
  {52, 1, 1},
  {54, 2, 1},
  {55, 2, 1},
  {56, 2, 1},
  {57, 2, 1},
  {58, 1, 0},
  { -1, -1, -1},
};

const unsigned long TXdialFreq [] =
{
  40620000,
  40680000,
  0
};

// include the library code:
#include <Wire.h>
#include <EEPROM.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"


// Set up MCU pins
#define InterruptPin             2
#define InterruptPin3            3
#define endCalibrate             8
#define NanoLED                 13
#define XmitLED                 14
#define FT8_XmitLED             15
#define On_Off_LED              16
#define pushButton2              7

// Set sI5351A I2C address
#define Si5351A_addr          0x60


// Define OLED address
// #define I2C_ADDRESS           0x3D // May be required for different oled display

#define I2C_ADDRESS            0x3C


// initialize oled display
SSD1306AsciiAvrI2c oled;

// Define Si5351A register addresses
#define CLK_ENABLE_CONTROL       3
#define CLK0_CONTROL            16
#define CLK1_CONTROL            17
#define CLK2_CONTROL            18
#define SYNTH_PLL_A             26
#define SYNTH_PLL_B             34
#define SYNTH_MS_0              42
#define SYNTH_MS_1              50
#define SYNTH_MS_2              58
#define SSC_EN                 149
#define PLL_RESET              177
#define XTAL_LOAD_CAP          183


// configure variables
bool TransmitFlag = false, suspendUpdateFlag = false;
bool toggle = false, transmit_toggle = false, toggle_LED;
int tcount2, TXcount, TXband;
int hours, minutes, seconds, startCount = 0, ttffsec = 0, testpps = 1;
int character, T_R_period, byteGPS = -1;
unsigned long XtalFreq = 25000000, tcount = 2, time_now = 0, LEDtimer, mult = 0;
char StartCommand[7] = "$GPGGA", StartCommand2[7] = "$GPRMC", buffer[100] = "", EW, NS;
byte Buffer[10], bufCounter, tempBuf, TransmitMode, tcount3, mode;
int IndiceCount = 0, StartCount = 0, counter = 0, indices[13], validGPSflag = 0;
static bool s_GpsOneSecTick = false;


// Load WSPR symbol frequency offsets
int OffsetFreq[4] = { -219, -73, 73 , 219};

// Load WSPR symbol length
//unsigned long SymbolLength = 683;

// Load FT8 symbol frequency offsets
int FT8_OffsetFreq[8] {0, 625, 1250, 1875, 2500, 3125, 3750, 4375};

// Load FT8 symbol length
//unsigned long FT8_SymbolLength = 159;

/*----------------------------------------------------------------------------------------------------
  The application to generate the data for “WSPRsymbols” is called WSPRMSG.exe and is found at
  <https://github.com/W3PM>.  Download this file into a convenient directory.
  Open the file and enter your callsign, grid locator, and power (dBm) as prompted.
  "WSPRsymbols" will create a file named "WSPRMSG.txt" in the same directory that "WSPRsymbols"
  resides. Cut and paste the symbol message data to replace the variable’s data.
  ----------------------------------------------------------------------------------------------------
*/

// Load WSPR channel symbols, call sign, grid square and RF power
// WSPR Data Packed 4 symbols per byte - MSBits first
// WZ9ZZZ AA99 37 (Generated from 'WSPRMSG.exe' W3PM January 2013)
//
const int  WSPRsymbols[162] = {
  3, 3, 0, 0, 0, 2, 0, 2, 3, 2, 2, 0, 3, 3, 1, 2, 2, 0, 1, 2, 0, 1, 2, 1, 1, 1, 1, 2, 2, 2, 0, 0,
  0, 2, 1, 2, 0, 1, 0, 1, 2, 0, 0, 2, 0, 2, 3, 2, 1, 3, 0, 0, 3, 1, 2, 1, 0, 2, 2, 1, 3, 2, 1, 2,
  2, 2, 0, 1, 3, 2, 1, 2, 1, 2, 1, 0, 3, 0, 2, 1, 0, 2, 3, 0, 3, 3, 0, 0, 0, 1, 3, 2, 3, 0, 1, 2,
  0, 0, 1, 2, 2, 0, 0, 2, 1, 2, 2, 1, 2, 0, 3, 1, 1, 0, 3, 3, 0, 0, 1, 1, 2, 1, 0, 0, 0, 3, 3, 3,
  2, 2, 2, 0, 2, 1, 2, 1, 0, 2, 1, 1, 0, 0, 0, 2, 0, 2, 2, 3, 1, 2, 1, 2, 1, 1, 2, 0, 0, 3, 3, 2,
  0, 0,
};


/*
   ----------------------------------------------------------------------------------------------------
    Open Windows Command Prompt. Navigate to directory C:\WSJTX\bin>. Enter the folling command
    using your own callsign and grid squre enclosed in quotes as follows:

    ft8code "CQ WZ9ZZZ AA99"  > FT8_message.txt"

    This will create a file named "FT8_message.txt" in C:\WSJTX\bin. Open the "FST4W_message.txt"
    file into a text editor. Separate the "Channel symbols:" data with commas. Replace the following
    data with the comma separated data:
  ----------------------------------------------------------------------------------------------------
*/
// Load FT8 channel symbols for "CQ WZ9ZZZ AA99"
const byte FT8symbols[79] = {
  3, 1, 4, 0, 6, 5, 2, 0, 0, 0,
  0, 0, 0, 0, 0, 1, 1, 4, 6, 4,
  7, 3, 1, 1, 3, 7, 0, 0, 0, 2,
  0, 4, 3, 2, 1, 7, 3, 1, 4, 0,
  6, 5, 2, 0, 5, 7, 5, 1, 4, 0,
  7, 1, 7, 0, 0, 7, 4, 3, 5, 7,
  3, 0, 5, 4, 7, 4, 1, 7, 2, 0,
  0, 7, 3, 1, 4, 0, 6, 5, 2
};





//----------------------------------------------------------------------------------------------------
//                         Sketch interrupts follow:
//----------------------------------------------------------------------------------------------------

//***********************************************************************
// This interrupt is used for Si5351 25MHz crystal frequency calibration
// Called every second by from the GPS 1PPS to Nano pin D2
//***********************************************************************
void Interrupt()
{
  s_GpsOneSecTick = true;                         // New second by GPS.
  tcount++;
  tcount2++;
  if (tcount == 4)                                // Start counting the 2.5 MHz signal from Si5351A CLK0
  {
    TCCR1B = 7;                                   //Clock on falling edge of pin 5
  }
  else if (tcount == 44)                         //Total count is = XtalFreq x 4 - stop counting
  {
    TCCR1B = 0;                                   //Turn off counter
    XtalFreq = (mult * 0x10000 + TCNT1) / 4ULL;   //Calculate corrected XtalFreq
    TCNT1 = 0;                                    //Reset count to zero
    mult = 0;
    tcount = 0;                                   //Reset the seconds counter
  }

  digitalWrite(NanoLED, HIGH);
  if (toggle_LED == false)
  {
    digitalWrite(NanoLED, LOW);
  }
  toggle_LED = !toggle_LED;
}


//******************************************************************
// Timer 1 overflow intrrupt vector.
// Called upon counter overflow from timer 1
//******************************************************************
ISR(TIMER1_OVF_vect)
{
  mult++;                                          //Increment multiplier
  TIFR1 = (1 << TOV1);                             //Clear overlow flag
}


//******************************************************************
// Transmit status interrupt follows:
// Displays WSPR transmitter ON/OFF status
//
//******************************************************************
void TransmitStatus()
{
  if (suspendUpdateFlag == false)
  {
    transmit_toggle = !transmit_toggle;
    if (transmit_toggle == false)
    {
      TransmitFlag = false;
      digitalWrite(On_Off_LED, LOW);
      oled.setCursor(0, 0);
      oled.print(F("XMITTER: OFF"));
    }
    else
    {
      TransmitFlag = true;
      digitalWrite(On_Off_LED, HIGH);
      oled.setCursor(0, 0);
      oled.print(F("XMITTER: ON "));
    }
  }
}




//----------------------------------------------------------------------------------------------------
//                         Initial sketch setup follows:
//----------------------------------------------------------------------------------------------------
void setup()
{
  Wire.begin();                                 // join I2C bus

  Serial.begin(9600);  // GPS receiver baud rate - DNB

  // NOTE: Change 9600 to 4800 for the Lantronix A2200-A GPS IC

  // Initialize the Si5351
  Si5351_write(XTAL_LOAD_CAP, 0b11000000);      // Set crystal load to 10pF
  Si5351_write(CLK_ENABLE_CONTROL, 0b00000000); // Enable CLK0, CLK1 and CLK2
  Si5351_write(CLK0_CONTROL, 0b01001111);       // Set PLLA to CLK0, 8 mA output, INT mode
  Si5351_write(CLK1_CONTROL, 0b01101111);       // Set PLLB to CLK1, 8 mA output, INT mode
  Si5351_write(CLK2_CONTROL, 0b01101111);       // Set PLLB to CLK2, 8 mA output, INT mode
  Si5351_write(PLL_RESET, 0b10100000);          // Reset PLLA and PLLB
  Si5351_write(SSC_EN, 0b00000000);             // Disable spread spectrum


  //Set up Timer1 as a frequency counter - input at pin 5
  TCCR1B = 0;                                    //Disable Timer5 during setup
  TCCR1A = 0;                                    //Reset
  TCNT1  = 0;                                    //Reset counter to zero
  TIFR1  = 1;                                    //Reset overflow
  TIMSK1 = 1;                                    //Turn on overflow flag

  // Inititalize 1 Hz input pin
  pinMode(InterruptPin, INPUT);
  digitalWrite(InterruptPin, LOW);              // internal pull-up enabled

  // Set pin D2 for external 1 Hz interrupt input
  attachInterrupt(digitalPinToInterrupt(InterruptPin), Interrupt, FALLING);


  // Set pin D3 for external transmitter on/off interrupt input
  pinMode(InterruptPin3, INPUT);
  digitalWrite(InterruptPin3, HIGH);           // internal pull-up enabled
  attachInterrupt(digitalPinToInterrupt(InterruptPin3), TransmitStatus, FALLING);


  // Set up push button
  pinMode(pushButton2, INPUT);
  digitalWrite(pushButton2, HIGH);      // internal pull-up enabled

  // Set up LEDs
  pinMode(NanoLED, INPUT);
  digitalWrite(NanoLED, HIGH);           // internal pull-up enabled
  pinMode(XmitLED, OUTPUT);              // Use with dropping resistor on pin D14
  digitalWrite(XmitLED, LOW);
  pinMode(FT8_XmitLED, OUTPUT);          // Use with dropping resistor on pin D15
  digitalWrite(FT8_XmitLED, LOW);
  pinMode(On_Off_LED, OUTPUT);           // Use with dropping resistor on pin D16
  digitalWrite(On_Off_LED, LOW);

  TCCR1B = 0;                             //Disable Timer1

  // Enable the Si5351
  Si5351_write(CLK_ENABLE_CONTROL, 0b00000010); // Enable CLK0 and CLK2 - disable CLK1

  // Set CLK0 to 2.5 MHz for autocalibration
  si5351aSetFreq(SYNTH_MS_0, 2500000UL, 0);

  // Set CLK2 to 2.5 MHz for frequency stabilization
  si5351aSetFreq(SYNTH_MS_2, 2500000UL, 0);

  // Set oled font size and type
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(fixed_bold10x15);

  // Setup OLED initial display
  oled.clear();
  oled.setCursor(0, 0);
  oled.println(F("WAITING FOR"));
  oled.setCursor(12, 2);
  oled.println(F("VALID GPS"));
  oled.setCursor(36, 4);
  oled.println(F("DATA"));

  TransmitFlag = false;
  digitalWrite(On_Off_LED, LOW);

  // Store time reference for sketch timing i.e. delays, EEPROM writes, display toggles
  // The Arduino "delay" function is not used because of critical timing conflicts
  time_now = millis();

}  //End Setup



//******************************************************************
// Loop starts here:
// Loops consecutively to check MCU pins for activity
//******************************************************************
void loop()
{
  GPSprocess();
  if (s_GpsOneSecTick )
  {
    s_GpsOneSecTick = false;

    if ((validGPSflag == 1 | validGPSflag == 2)  & suspendUpdateFlag == false)
    {
      /*
        ----------------------------------------------------------------
         Multi mode transmissions set by a schedule begins here
        ----------------------------------------------------------------
              Multimode transmitting on an hourly schedule begins here

                1 = WSPR
                2 = FT8

               Schedule start time limitations:
               WSPR : any even numbered minute
               FT8  : any integer munute

              -----------------------------------------------------------------
      */
      if ((seconds == 0 | seconds == 30) & TransmitFlag == true)
      {
        for (int timeslot = 0; timeslot < 60; timeslot++)
        {
          if (schedule [timeslot] [0] == -1) return;
          if ((schedule [timeslot] [0]) == minutes)
          {
            TXband = schedule [timeslot] [2];
            if (schedule [timeslot] [1] == 1 & seconds == 0 ) transmit();   // transmit WSPR
            if (schedule [timeslot] [1] == 2) if (seconds == 0 | seconds == 30) FT8transmit(); // FT8 mode
          }
        }
      }

      // ---------------------------------------------------------------------
      // Do the following when not transmitting
      // ---------------------------------------------------------------------
      setfreq();
      displayClock();
      if (TransmitFlag == false)
      {
        digitalWrite(On_Off_LED, LOW);
        oled.setCursor(0, 0);
        oled.print(F("XMITTER: OFF"));
      }
      else
      {
        digitalWrite(On_Off_LED, HIGH);
        oled.setCursor(0, 0);
        oled.print(F("XMITTER: ON "));
      }
    }
  }
}


//******************************************************************
//  Selection Header:
//  Used to display the common header used in some menu selections
//
//  Called by all edit funtions
//******************************************************************
void SelectionHeader()
{
  oled.clear();
  oled.setCursor(0, 4);
  oled.print("1&4 - edit");
  oled.setCursor(0, 6);
  oled.print("3 - save");
}


//******************************************************************
// Alternate delay function follows:
// altDelay is used because the command "delay" causes critical
// timing errors.
//
// Called by all functions
//******************************************************************
unsigned long altDelay(unsigned long delayTime)
{
  time_now = millis();
  while (millis() < time_now + delayTime) //delay 1 second
  {
    __asm__ __volatile__ ("nop");
  }
}



//******************************************************************
// displayClock follows:
// Displays UTC time
//
// Called by loop()
//******************************************************************
void displayClock()
{
  oled.setCursor(0, 4);
  oled.print(F(" "));

  if (hours < 10) oled.print(F("0"));
  oled.print(hours);
  oled.print(F(":"));

  if (minutes < 10) oled.print(F("0"));
  oled.print(minutes);
  oled.print(F(":"));

  if (seconds < 10) oled.print(F("0"));
  oled.print(seconds);
  oled.print(F("   "));
}



//******************************************************************
// Set transmit frequency function follows:
// Calculates the frequency data to be sent to the Si5351 clock generator
// and displays the frequency on the olded display
//
// Called by bandSelect() and transmit()
//******************************************************************
void setfreq()
{
  unsigned long  frequency = TXdialFreq [TXband] + TXoffset; // Temporarily store Freq_1

  oled.setCursor(0, 2);
  char buf[11];
  if (frequency >= 1000000UL)
  {
    int MHz = int(frequency / 1000000UL);
    int kHz = int ((frequency - (MHz * 1000000UL)) / 1000UL);
    int Hz = int (frequency % 1000UL);
    snprintf(buf, sizeof(buf), "%2u,%03u,%03u", MHz, kHz, Hz);
  }

  else if (frequency >= 1000UL & frequency < 1000000UL)
  {
    int kHz = int (frequency / 1000UL);
    int Hz = int (frequency % 1000UL);
    snprintf(buf, sizeof(buf), "%6u,%03u", kHz, Hz);
  }
  else if (frequency < 1000UL)
  {
    int Hz = int (frequency);
    snprintf(buf, sizeof(buf), "%10u", Hz);
  }
  oled.print(buf);
}


//******************************************************************
// Transmit algorithm follows:
// Configures data and timing and then sends data to the Si5351
//
// Called by loop()
//******************************************************************
void transmit()
{
  TransmitFlag = false;
  //int SymbolCount = 162; // WSPR symbol count
  //time_now = millis();
  suspendUpdateFlag = true;
  if (FreqHopTX == true) // Enables in-band TX frequency hopping in incremental 10Hz steps
  {
    TXoffset = TXoffset + 10UL;
    if (TXoffset > 1550UL) TXoffset = 1440UL;
  }
  Si5351_write(CLK_ENABLE_CONTROL, 0b00000100); // Enable CLK0 CLK1 - disable CLK2
  setfreq();
  oled.setCursor(0, 4);
  oled.print(F(" XMITTING "));
  oled.setCursor(0, 6);
  oled.print(F("            "));
  oled.setCursor(0, 6);
  oled.print(F("MODE:  WSPR "));
  digitalWrite(XmitLED, HIGH);
  unsigned long currentTime = millis();
  for (int count = 0; count < 162; count++)
  {
    unsigned long timer = millis();
    si5351aSetFreq(SYNTH_MS_1, (TXdialFreq [TXband] + TXoffset), OffsetFreq[WSPRsymbols[count]]); // WSPR mode
    while ((millis() - timer) <= 683)
    {
      __asm__("nop\n\t");
    };
  }
  Si5351_write(CLK_ENABLE_CONTROL, 0b00000010); // Enable CLK0 CLK2 - disable CLK1 (drift compensation)
  si5351aSetFreq(SYNTH_MS_2, 2500000UL, 0);        // CLK2 is enabled to balance thermal drift between transmissions
  digitalWrite(XmitLED, LOW);
  suspendUpdateFlag = false;
  TransmitFlag = true;
  seconds = 53;// Time is not updated during transmit. This prevents loopback to transmit
}



//******************************************************************
// FT8 Transmit algorithm follows:
// Configures data and timing and then sends data to the Si5351
//
// Called by loop()
//******************************************************************
void FT8transmit()
{
  //delay(1000);
  TransmitFlag = false;
  //time_now = millis();
  suspendUpdateFlag = true;
  if (FT8_FreqHopTX == true) // Enables in-band TX frequency hopping in incremental 20Hz steps
  {
    FT8_TXoffset = FT8_TXoffset + 100UL;
    if (FT8_TXoffset > 2440UL) FT8_TXoffset = 440UL;
  }
  Si5351_write(CLK_ENABLE_CONTROL, 0b00000100); // Enable CLK0 CLK1 - disable CLK2
  setfreq();
  oled.setCursor(0, 4);
  oled.print(F(" XMITTING "));
  oled.setCursor(0, 6);
  oled.print(F("MODE:  FT8  "));
  digitalWrite(FT8_XmitLED, HIGH);
  unsigned long currentTime = millis();
  for (int count = 0; count < 79; count++)
  {
    unsigned long timer = millis();
    si5351aSetFreq(SYNTH_MS_1, (TXdialFreq [TXband] + FT8_TXoffset) , FT8_OffsetFreq[FT8symbols[count]]); // FT8 mode
    while ((millis() - timer) <= 159)
    {
      __asm__("nop\n\t");
    };
  }
  Si5351_write(CLK_ENABLE_CONTROL, 0b00000010); // Enable CLK0 CLK2 - disable CLK1 (drift compensation)
  si5351aSetFreq(SYNTH_MS_2, 2500000UL, 0);        // CLK2 is enabled to balance thermal drift between transmissions
  digitalWrite(FT8_XmitLED, LOW);
  suspendUpdateFlag = false;
  TransmitFlag = true;
  seconds = 53;// Time is not updated during transmit. This prevents loopback to transmit
}



//******************************************************************
//  GPS NMEA processing starts here
//  Just uses the $GPRMC NMEA message for getting the time
//  called by loop()
//  NOTE: The original code looked for $GPGGA as well as $GPRMC messages
//  The $GPGGA iformation is not needed for this particular version so
//  just the $GPRMC message is used for getting the time info
//******************************************************************

void GPSprocess()  //Modified version by DNB - K4CLE - only looking for the $GPRMC message
{
  byte temp, pinState = 0, i;
  if (Serial.available() > 0)
  {
    byteGPS = Serial.read();     // Read a byte of the serial port
    buffer[counter] = byteGPS;   // If there is serial port data, it is put in the buffer
    counter++;
    if (byteGPS == 13) {         // If the received byte is = to 13, end of transmission
      // The previous GPS message should have sent a carriage return, i.e., ASCII 13

      // looking for just the NMEA $GPRMC message here
      IndiceCount = 0;
      StartCount = 0;  //StartCount used to count each character
      for (int i = 1; i < 7; i++) { // Verifies if the received command starts with $GPRMC
        if (buffer[i] == StartCommand2[i - 1]) {  //StartCommand2(7) is $GPRMC
          StartCount++; // continue checking each character
        }
      }
      if (StartCount == 6) {     // If yes, continue and process the data

        // NOTE: the above number has to be changed from the original 6 to a 5
        // for the uBlox NEO - M8 GPS IC
        // for earlier versions of uBlox receivers, StartCount == 6, DNB

        // Process the message here
        for (int i = 0; i < 100; i++) {
          if (buffer[i] == ',') { // check next for the position of the  "," separator
            indices[IndiceCount] = i;
            IndiceCount++;
          }
          if (buffer[i] == '*') { // ... and the "*" indicates got to end of the message
            indices[12] = i;
            IndiceCount++;
          }
          // Now, load just the time data portion of the RMC message
          temp = indices[0];
          // Subtract 48 from each number to get rid of the ASCII description
          hours = (buffer[temp + 1] - 48) * 10 + buffer[temp + 2] - 48;
          minutes = (buffer[temp + 3] - 48) * 10 + buffer[temp + 4] - 48;
          seconds = (buffer[temp + 5] - 48) * 10 + buffer[temp + 6] - 48;
          temp = indices[1];

          // Now we have to see if this is a valid $GPRMC message
          // The valid letter is either an ASCII "A" which is valid
          // or a AnSCII "V" for invalid
          if (buffer[temp + 1] == 65) {  // 65 is ASCII letter "A"
            validGPSflag = 1;  //YEA!! it's good
          }
          else  // Then it was not an ASCII "A", must be a "V" instead
          {
            validGPSflag = 0 ;  //don't use this $GPRMC message
          }
        }
      }
      counter = 0;   // Reset the buffer with blanks
      for (int i = 0; i < 100; i++) buffer[i] = ' ';

    } //End if byteGPS == 13
  } //End Serial.Available
} //End GPSprocess



//******************************************************************
//  Si5351 processing follows:
//  Generates the Si5351 clock generator frequency message
//
//  Called by sketch setup() and transmit()
//******************************************************************
void si5351aSetFreq(int synth, unsigned long  frequency, int symbolOffset)
{
  unsigned long long  CalcTemp;
  unsigned long PLLfreq, divider, a, b, c, p1, p2, p3, PLL_P1, PLL_P2, PLL_P3;
  byte dividerBits = 0;
  int PLL, multiplier = 1;
  if (synth == SYNTH_MS_0) PLL = SYNTH_PLL_A;
  else PLL = SYNTH_PLL_B;

  frequency = frequency * 100UL + symbolOffset;
  c = 0xFFFFF;  // Denominator derived from max bits 2^20

  divider = 90000000000ULL / frequency;
  while (divider > 900UL) {           // If output divider out of range (>900) use additional Output divider
    dividerBits++;
    divider = divider / 2UL;
    multiplier = multiplier * 2;
    //multiplier = dividerBits << 4;
  }

  dividerBits = dividerBits << 4;
  if (divider % 2) divider--;
  //  PLLfreq = (divider * multiplier * frequency) / 10UL;

  unsigned long long PLLfreq2, divider2 = divider, multiplier2 = multiplier;
  PLLfreq2 = (divider2 * multiplier2 * frequency) / 100UL;
  PLLfreq = PLLfreq2;

  a = PLLfreq / XtalFreq;
  CalcTemp = PLLfreq2 - a * XtalFreq;
  CalcTemp *= c;
  CalcTemp /= XtalFreq ;
  b = CalcTemp;  // Calculated numerator


  p1 = 128UL * divider - 512UL;
  CalcTemp = 128UL * b / c;
  PLL_P1 = 128UL * a + CalcTemp - 512UL;
  PLL_P2 = 128UL * b - CalcTemp * c;
  PLL_P3 = c;


  // Write data to PLL registers
  Si5351_write(PLL, 0xFF);
  Si5351_write(PLL + 1, 0xFF);
  Si5351_write(PLL + 2, (PLL_P1 & 0x00030000) >> 16);
  Si5351_write(PLL + 3, (PLL_P1 & 0x0000FF00) >> 8);
  Si5351_write(PLL + 4, (PLL_P1 & 0x000000FF));
  Si5351_write(PLL + 5, 0xF0 | ((PLL_P2 & 0x000F0000) >> 16));
  Si5351_write(PLL + 6, (PLL_P2 & 0x0000FF00) >> 8);
  Si5351_write(PLL + 7, (PLL_P2 & 0x000000FF));


  // Write data to multisynth registers
  Si5351_write(synth, 0xFF);
  Si5351_write(synth + 1, 0xFF);
  Si5351_write(synth + 2, (p1 & 0x00030000) >> 16 | dividerBits);
  Si5351_write(synth + 3, (p1 & 0x0000FF00) >> 8);
  Si5351_write(synth + 4, (p1 & 0x000000FF));
  Si5351_write (synth + 5, 0);
  Si5351_write (synth + 6, 0);
  Si5351_write (synth + 7, 0);

}


//******************************************************************
// Write I2C data function for the Si5351A follows:
// Writes data over the I2C bus to the appropriate device defined by
// the address sent to it.

// Called by sketch setup, transmit(), si5351aSetFreq, and
// si5351aStart functions.
//******************************************************************
uint8_t Si5351_write(uint8_t addr, uint8_t data)
{
  Wire.beginTransmission(Si5351A_addr);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}
