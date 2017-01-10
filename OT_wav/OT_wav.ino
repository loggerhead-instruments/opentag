/* Loggerhead Instruments OpenTagGAMPT2 v2.8
   Release 30 April 2015
   Copyright 2015 by David Mann

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
 
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.  

    You should have received a copy of the GNU General Pub lic License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
Records z-axis of accelerometer to a 16-bit binary file with no header.
Sample rate is fixed at  .

*/
#include <stdint.h>
#include <SdFat.h> // http://code.google.com/p/sdfatlib/  //note that the SD library that comes with Arduino does not support file timestamps, so not using it
#include <SdFatUtil.h>
#include <Wire.h>
#include <FlexiTimer2.h>

// SD chip select pin
const uint8_t CS = SS;

byte accelint;
byte SD_POW=5;
byte LED_GRN=4;
byte LED_RED=A3;
boolean toggle = 0;

// I2C Addresses
int AccelAddress = 0x53;  //with pin 12 grounded; board accel

SdFat card;
SdFile file;

typedef struct {
    char    rId[4];  // 4 bytes
    unsigned long rLen; // 4 bytes
    char    wId[4];    // 4 bytes
    char    fId[4];    // 4 bytes
    unsigned long    fLen;      // 4 bytes
    unsigned short nFormatTag; // 2 bytes
    unsigned short nChannels;  // 2 bytes
    unsigned long nSamplesPerSec;  // 4 bytes
    unsigned long nAvgBytesPerSec; // 4 bytes
    unsigned short nBlockAlign;   // 2 bytes
    unsigned short  nBitsPerSamples; // 2 bytes
    char    dId[4]; // 4 bytes
    unsigned long    dLen; // 4 bytes
} HdrStruct;
HdrStruct wav_hdr;

char filename[12];
static unsigned long count=0;

#define BUFFERSIZE 512 
byte buffer[BUFFERSIZE]; //Double buffer used to store IMU sensor data before writes in bytes

byte time2write=0;  //=0 no write; =1 write first half; =2 write second half
int halfbuf=BUFFERSIZE/2;
int bufferpos=0; //current position in double buffer

boolean firstwritten;
float srate = 1400.0;
unsigned long newfilebufs = (unsigned long) (300.0 / (halfbuf/srate));
unsigned int RTCcounter=0;
boolean checkstop = 0;
boolean introperiod=1;  //flag for introductory period; used for keeping LED on for a little while
boolean LEDSON=0;
unsigned int counter=0;
unsigned int nbufs=0;

void(* resetFunc) (void) = 0;//declare reset function at address 0

//
// ------  Setup   --------
//
void setup() {  
  pinMode(SD_POW, OUTPUT);      
  digitalWrite(SD_POW, HIGH);  //turn on power to SD card.  High for OpenTag.
  system_start();
  TWBR=400000L;
  Wire.begin();
  Accel_Init(AccelAddress,srate, 0, 0);
  accelint = readAccelInt(AccelAddress); //read to clear accelerometer interrupts
  Read_Accel(AccelAddress);
  count=0;
  FlexiTimer2::set(1, 1.0/srate, flash);
  FlexiTimer2::start();
}
//
// ------  Main Loop   --------
//
void loop() { 
      if(time2write==1)
      {
        if(LEDSON | introperiod) digitalWrite(LED_GRN,HIGH);        
        if(file.write(buffer, halfbuf)==-1) resetFunc(); 
        time2write=0;
        nbufs+=1;
        if(LEDSON | introperiod) digitalWrite(LED_GRN,LOW);
      }
      if(time2write==2)
      {    
        if(file.write((const void*)&buffer[halfbuf], halfbuf)==-1) resetFunc();
        time2write=0;
        nbufs+=1;
       }    

      // new file every n nbufs
      if(nbufs >= newfilebufs * 2)
      {
        introperiod = 0;
        file.close();
        FileInit();
        nbufs = 0;
      }
      
      // check for short for stop
      if(checkstop==1)
      {    
        checkstop = 0;
        int stopval = analogRead(A2); //Stop pads shorted?
        if(stopval<20) 
        {
            FileStop();
            digitalWrite(LED_RED, HIGH);
            delay(30000);
            digitalWrite(LED_RED, LOW);
        }
    }
}

// increment buffer position by 1 byte.  check for end of buffer
void incrementbufpos(){
   boolean overflow;
   bufferpos++;
   if(bufferpos==BUFFERSIZE)
   {
     bufferpos=0; 
     time2write=2;  // set flag to write second half
     firstwritten=0; 
 }
 
  if((bufferpos>=halfbuf) & !firstwritten)  //at end of first buffer
  {
    time2write=1; 
    firstwritten=1;  //flag to prevent first half from being written more than once; reset when reach end of double buffer
  }
}

/********************************************************************
***        Master Interrupt Routine to Read Sensors               ***
********************************************************************/
void flash(){  
  RTCcounter++;
  if(RTCcounter>=srate) 
  {
    RTCcounter=0;
    checkstop = 1;
  //  toggle = !toggle;
  //  digitalWrite(LED_GRN,toggle);
  }   
    sei();
    Read_Accel(AccelAddress);
}

void system_init(){  
    pinMode(LED_GRN, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(2, INPUT); //Arduino Interrupt2
    pinMode(3, INPUT); //Arduino Interrupt1
    pinMode(A2, INPUT);  //used to detect stop
    digitalWrite(A2, HIGH);
    digitalWrite(LED_RED,LOW);
    digitalWrite(LED_GRN,LOW);
    analogReference(DEFAULT);
}

void SDcard_init(){
    pinMode(SD_POW, OUTPUT);      
    digitalWrite(SD_POW, HIGH);  //turn on power to SD card.  High for OpenTag.
    delay(200);
    if (!card.begin(CS, SPI_FULL_SPEED)) resetFunc();
}

void system_start(){
   system_init();
   SDcard_init();
   FileInit();
}

void FileInit()
{
   // open file 
   count += 1;
   sprintf(filename,"%d.wav",count);
   
  //Create a file. If the file already exists, increment counter and try again.
  while(!file.open(filename, O_CREAT | O_EXCL | O_WRITE))
  {
   count+=1; 
   sprintf(filename,"%d.wav",count);
   if(count>1000000) resetFunc();
   digitalWrite(LED_RED,HIGH);
  }
  digitalWrite(LED_RED,LOW);
   //intialize .wav file header
    sprintf(wav_hdr.rId,"RIFF");
    wav_hdr.rLen=36;
    sprintf(wav_hdr.wId,"WAVE");
    sprintf(wav_hdr.fId,"fmt ");
    wav_hdr.fLen=0x10;
    wav_hdr.nFormatTag=1;
    wav_hdr.nChannels=1;
    wav_hdr.nSamplesPerSec=(unsigned long) srate;
    wav_hdr.nAvgBytesPerSec=(unsigned long) srate*2;
    wav_hdr.nBlockAlign=2;
    wav_hdr.nBitsPerSamples=16;
    sprintf(wav_hdr.dId,"data");
    wav_hdr.rLen = 36 + (newfilebufs * BUFFERSIZE);
    wav_hdr.dLen = newfilebufs * BUFFERSIZE;
  
    file.write((uint8_t *)&wav_hdr, sizeof(wav_hdr));
}

void FileStop(){
  introperiod = 0;
  file.close();
}

