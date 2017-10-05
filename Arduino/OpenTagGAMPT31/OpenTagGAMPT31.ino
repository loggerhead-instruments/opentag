/* Loggerhead Instruments OpenTagGAMPT2 v23.1
   Release 5 October 2017
   Copyright 2017 by David Mann

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Additional software and schematics available at http://www.loggerheadinstruments.com

// Some code derived from SparkFun SF9DOF_AHRS by Doug Weibel and Jose Julio
// Based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel

// Version 3.1.1
// fixed bug with depth calculated as bars and not meters

// Version 3.1
// 1. Fixed bug with always recording all sensors

// Version 3.0
// 1. Added duty cycle record to WT command
// 2. Removed wake on day of week
// 3. Removed LED for burnwire
// 4. Green LED flashes only for first half of buffer write

// Version 2.9
// 1. Added command 'BT':  HH:MM from power on until burnwire goes
// 2. If LEDS_ON will red LED will flash when burn wire is triggered
// 3. Script code: IS (interval seconds for I2CPOW).  Set to 0 for continuous on.
//    second value is depth (<depth will turn on I2CPOW--overriding interval).


// Version 2.8
// 1. Default to new file every hour
// 2. Code is ND to make new file every day
// 3. Set lastday and lasthour to current RTC at start
// 4. Add duty cycle (x seconds on/y seconds off) for I2Cpow, so can use to power VHF
//    VHF interfere with Remora recordings
//    default: IS 60 1   (60 s on/60 s off;  on if depth< 1m)  Numbers must be integers

// Version 2.7 
// 1. Pressure/temperature calibration will now overwrite existing file.

// Version 2.6 Updates
// 1. Added day of week to waketime (WT).  Day of week is 1-7.  If 0 is passed, day of week alarm is not used.

// Version 2.5 Updates
// 1. Added new file every hour

// Version 2.4 Updates
// 1. Removed WDT
// 2. Removed Salt Detection
// 3. Removed External Accelerometer
// 4. Restructured Main Loop to transition between, Sleep, Wake, Record states.

// Version 2.3 Updates
// 1. Added support for gyro range from script file (e.g. GY 3 = +/-2000 deg/s).  Setting stored in StoreType in SID_SPEC

// Version 2.2 Updates
// 1. Increment counter by 24 when can't open file.
// 2. Added check for correct file write.  If fails, restarts code.
// 3. Clear WDT while checking for new file.
// 4. WDTflag must be set = 1 to enable WDT.  WDT incompatible with Arduino bootloader.

// Version 2.1 Updates
// 1. Added Watchdog timer enable to reset if hangs
// 2. Removed sleep on init

// Version 2.0 Updates
// 1. Code for magnetometer reading changed to requesting individual reads, rather than having magnetometer in streaming  mode.  This eliminates occassional spikes in magnetometer readings.
// 2. Added Command (CP) to set Clock Prescaler, to change the speed of the AtMega328p.  This can have significant power savings, but have to check for overflow.
// 3. Burn flag (for underwater burn wire) implemented.
// 4. Delay start implemented.
// 5. New files start on the hour, so file duration does not change with sample rate.
// 6. When LEDs disabled, they run for the first file.
// 7. Check for stop every 1 second
// 8. Enable Sleep on gyroscope and magnetometer if not being used
// 9. Start interrupt timer for reading sensors after first file created (so don't get overflow before start)
// 10.  Added variable low-pass filter setting on gyroscope

//Notes
// 0. Written with Arduino v0022.  Note Arduino v1.0 will not work because of issues with changes to Wire.h that produce compile errors.  Waiting for next update which will fix this.
// 1. OpenTag (Windows software) is used to create the Default.txt file that is read to set the time and sample rates.
// 2. The current time will only be updated from the schedule if it is more recent than the current time held by the RTC, unless Force Reset.
// 3. Orientation of x,y,z follows that of accelerometer on board (readings of magnetometer correct to same orientation when read).
// 4. A7 analog channel has been commented out of this version.
// 5. Calculations of Pressure involve 64-bit math which chews up a lot of sketch size.  So, just saving raw 24-bit pressure and temp reads along with coefficients to calc P and T in post-processing. 
// 6. Burn time is not implemented
// 7. For programming with Sparkfun FTDI Basic GND and CTL must be shorted.  There is no corresponding pin for CTL on OpenTag.
// 8. This code is close to filling code space.  So Serial.Prints are commented.

// 9. Data are streamed to one file at a time.  The file structure follows the datafile.h header, and is generally as follows:
// DF_HEAD: At start of each file. Contains start time, code version number, and values for lat, lon, depth (altitude)
// SID_SPEC: Follows DF_HEAD.  May be more than one SID_SPEC depending on which sensors are being saved.  Contains info on sample period, sensor types, number of channels
// SID_REC: As data buffers are filled, they are written to the file.  Each data chunk starts with a SID_REC which indicates which SID_SPEC type it is, sensors stored, and the number of bytes recorded since the start of recording.

/*
// Revision History
//  beta dfh.Version=10000;
//  dfh.Version=10001; 13 June 2012
//      Added code so nbuffers not incremented in pressure or ADC loops if IMU being saved.  Keeps things aligned.
// dfh.Version=10003; 22 November v1.1; changed accelerometer from +/2g max scale to +/16g max scale
// dfh.Version=10020; 23 Feb 2012; added prescaler.h to allow changes to clock speed
                                   check if stop pads shorted every two seconds, instead of as a function of buffer length
                                   fixed error where would set month for seconds when using TM
                                   fixed error in leap year RTC seconds UNIX time calcuation
                                   magnetometer placed in single measurement mode to save power
                                   
*/
// Sid_SPEC.SensorType Binary flag structure for sensors (sensor = bit shift)
// Analog ADC0 = 0
// temperature = 1
// Pressure = 2
// gyro = 3
// magnetometer = 4
// accelerometer internal = 5
// accelerometer external = 6

// Order Data are written to file (if present)
// "HYD1" HYDRO1, Accel Ext
// "PTMP" Pressure, Temp
// "INER" Accel Int, Comp, Gyro

#include <stdint.h>
#include <SdFat.h> 
#include <Wire.h>
#include <MsTimer2.h>  // Note: MsTimer2.cpp modified so it does not disable interrupts
#include <datafile.h>
#include <prescaler.h>
#include <avr/sleep.h>
#include <avr/power.h>

//#define SDA_PORT PORTC
//#define SDA_PIN 4
//#define SCL_PORT PORTC
//#define SCL_PIN 5
//
//#define I2C_TIMEOUT 100
//#define I2C_FASTMODE 1
//
//#include <SoftWire.h>
//#include <avr/io.h>
//SoftWire Wire = SoftWire();

// SD chip select pin
const uint8_t CS = 10;

byte state;
#define GO_TO_SLEEP 0
#define WAKE_UP 1
#define RECORDING 2

byte accelint;

int BURN=8;
int I2CPOW=9;
int SD_POW=5;
int LED_GRN=4;
int LED_RED=A3;
int HYDRO1=A0;
int PRESS=A6;
//int ACCEL_INT2=12;

// I2C Addresses
int AccelAddressInt = 0x53;  //with pin 12 grounded; board accel

// Default Flags...These can be changed by script
boolean accelflagint=1;  //flag to enable accelerometer; 
boolean compflag=1; //flag to enable 3d magnetometer (compass)
boolean gyroflag=0; //flag to enable gyro
byte gyrorange = 1; //+/- 500 degrees/s
boolean pressflag=1; //flag to enable pressure
boolean tempflag=1; //flag to enable temperature
boolean HYDRO1flag=0; // flag to record HYDRO1
boolean printflag=0; //flag to enable printing of sensor data to serial output
byte burnflag=0;  // flag to enable burn wire
byte clockprescaler=0;  //clock prescaler
boolean updatepressflag=0;  //=1 call UpdatePress();  takes 9 ms for conversion
boolean updatetempflag=0;  //=1 call UpdateTemp();  takes 9 ms for conversion
boolean alarmflag=0; //=1, wake up every HH:MM for HH:MM
boolean alarmstatus=0; //if woke by alarm alarmstatus=1
boolean motionflag=0; // =1 using motion from acceleromter to wake

SdFat sd;
SdFile file;

char filename[12];
static unsigned long count=0;
#define BUFFERSIZE 144 // used this length because it is divisible by 36 bytes (e.g. Aint,M,G)  //default 144  //IMU only; no ADC  288
//#define BUFFERSIZE 252 // used this length because it is divisible by 36 bytes (e.g. Aint,M,G)  //default 144  //IMU only; no ADC  288
byte buffer[BUFFERSIZE]; //Double buffer used to store IMU sensor data before writes in bytes

#define PTBUFFERSIZE 24
byte PTbuffer[PTBUFFERSIZE];  //Double buffer used to store Pressure and Temp at low rate. Note Press and Temp each 3 bytes.
byte time2write=0;  //=0 no write; =1 write first half; =2 write second half
byte time2writePT=0; 

byte halfbufPT=PTBUFFERSIZE/2;
int halfbuf=BUFFERSIZE/2;
int bufferpos=0; //current position in double buffer
byte bufferposPT=0;

boolean firstwritten;
boolean firstwrittenPT;

int speriod=10; // default master sample rate interrupt ms
int iperiod=20; // default sample period for motion sensors ms
int mscale_period=iperiod/speriod;  //number of speriods before sample motion sensors
int mscale_counter=0;

int PTperiod=1000; //Press Temp Sample Period in ms
int mscale_PTperiod=PTperiod/speriod;  //sample period for pressure/temperature sensor; number of speriods before sample pressure and temp
int mscale_PTcounter;
unsigned int RTCcounter=0;
boolean checkstop = 0;
unsigned int InactiveTime = 0;
int old_magnetom_x; //to keep track of changing magnetometer
int inactivity_threshold = 100; 
int inactivity_duration = 60; //seconds
byte threshold = 0; //accelerometer threshold

float i2cpowdepth = 1.0; // depth below which i2cpow on
boolean I2CPOW_STATE;

#define SECONDS_IN_MINUTE 60
#define SECONDS_IN_HOUR 3600
#define SECONDS_IN_DAY 86400
#define SECONDS_IN_YEAR 31536000
#define SECONDS_IN_LEAP 31622400
byte hour;
byte minute;
byte second;
byte year;
byte month;
byte date;

byte lastdate;  //for keeping track of when last file was created
byte lasthour; // for keeping track of when last file was created
boolean introperiod=1;  //flag for introductory period; used for keeping LED on for a little while
boolean neweveryday = 0;
boolean neweveryhour = 1;

TIME_HEAD nowtime;
TIME_HEAD burntime;
byte waketime_hour; //wake time (HH:MM)
byte waketime_minute;
ULONG burnInSeconds; //when setting burn time as duration from start
ULONG burntimesec;  //unix time for burn time
ULONG nowtimesec;  //unix time for current time
ULONG recsec; //seconds to record when using alarm
ULONG waketimesec; //unix time at wakeup (in seconds)

byte sleep_hour=0;
byte sleep_minute=0;

// interval between timer interrupts in microseconds
const uint16_t TICK_TIME_USEC = 1000;
byte LEDSON=1;
unsigned int counter=0;

int accel_x_int;
int accel_y_int;
int accel_z_int;
int magnetom_x;
int magnetom_y;
int magnetom_z;
int gyro_x;
int gyro_y;
int gyro_z;
int gyro_temp;

byte Tbuff[3];
byte Pbuff[3];
float depth = 1;

//Pressure and temp calibration coefficients
unsigned int PSENS; //pressure sensitivity
unsigned int POFF;  //Pressure offset
unsigned int TCSENS; //Temp coefficient of pressure sensitivity
unsigned int TCOFF; //Temp coefficient of pressure offset
unsigned int TREF;  //Ref temperature
unsigned int TEMPSENS; //Temperature sensitivity coefficient

// Header for dsg files
DF_HEAD dfh;
SID_SPEC SidSpec[SID_MAX];
SID_REC SidRec[SID_MAX];

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

void(* resetFunc) (void) = 0;//declare reset function at address 0

//
// ------  Setup   --------
//
void setup() {  
  pinMode(LED_GRN, OUTPUT);
  pinMode(SD_POW, OUTPUT);      
  digitalWrite(SD_POW, HIGH);  //turn on power to SD card.  High for OpenTag.
  I2C_Init();
  //Serial.begin(57600); 

  SDcard_init();
  LoadScript(file);
  
  mscale_period=iperiod/speriod;
  mscale_PTperiod=PTperiod/speriod/(pressflag+tempflag);
  
  system_init();

  for (int n=0; n<SID_MAX; n++)
  {
    SidSpec[n].SID=0;
    SidRec[n].nbytes=0;
    SidRec[n].nbytes_2=0;
  }

  dfh.Version=10311;
  dfh.UserID=3333;
  sensor_init();
  if(pressflag|tempflag)
  {
    // Write out pressure and temperature calibration values to a file
   if(file.open("presstmp.cal", O_CREAT | O_TRUNC | O_WRITE))
   {
      Read_RTC();  //update time
      file.timestamp(T_WRITE,(uint16_t) year+2000,month,date,hour,minute,second);
      file.write(&PSENS,2);    
      file.write(&POFF,2);     
      file.write(&TCSENS,2);  
      file.write(&TCOFF,2);  
      file.write(&TREF,2);  
      file.write(&TEMPSENS,2);     
      file.close();
    }
  }
  count=0;
  Read_RTC();
  lastdate=date;
  lasthour=hour;

  if (burnflag == 2){
    updatetime(&nowtime);  //load into time structure
    nowtimesec=RTCToUNIXTime(&nowtime);  
    burntimesec = nowtimesec + burnInSeconds;
//    Serial.print("Time: ");
//    Serial.println(nowtimesec);
//    Serial.print("Burn: ");
//    Serial.println(burntimesec);
  }
    
  state = GO_TO_SLEEP;
  if (alarmflag==0 & motionflag==0)  //continuous record, don't sleep
      state = WAKE_UP;
      

}
//
// ------  Main Loop   --------
//
void loop() {
  
    if(state==GO_TO_SLEEP)
    {

      //Serial.println("S");
      // sensors off
      Gyro_Init(0,40);  //if gyroflag=0; will put to sleep if sensor present
      Compass_Init(0); //if compflag=0; will put in Idle mode if sensor present
      Accel_Init(AccelAddressInt, 12.5, 1, threshold); //12.5 Hz low power mode
      digitalWrite(SD_POW, LOW);  //turn off power to SD card. 
      digitalWrite(LED_GRN,LOW);
      trueDelay(50);
      system_sleep(); //go to sleep
      // ... asleep here ..  
      state = WAKE_UP; 
    }
    
    if(state==WAKE_UP)
    {
      //Serial.println("W");
      system_start();
      updatetime(&nowtime);  //load into time structure
      waketimesec=RTCToUNIXTime(&nowtime);
      state=RECORDING;
    }

    if(state==RECORDING)
    {
      if(time2write==1)
      {
       // if(LEDSON | introperiod) digitalWrite(LED_GRN,HIGH);        
        if(file.write(&SidRec[0],sizeof(SID_REC))==-1) resetFunc();  // make sure wrote; if not reboot
        if(file.write(buffer, halfbuf)==-1) resetFunc(); 
        UpdateSID_REC(0,halfbuf);  // update SID_REC with number of bytes written
        time2write=0;
     //   if(LEDSON | introperiod) digitalWrite(LED_GRN,LOW);
      }
      if(time2write==2)
      {       
        if(file.write(&SidRec[0],sizeof(SID_REC))==-1) resetFunc();
        if(file.write((const void*)&buffer[halfbuf], halfbuf)==-1) resetFunc();
        UpdateSID_REC(0,halfbuf);  // update SID_REC with number of bytes written      
        time2write=0;
       }    
      if(time2writePT==1)
      {
        if(LEDSON | introperiod) digitalWrite(LED_GRN,HIGH);
        if(file.write(&SidRec[2],sizeof(SID_REC))==-1) resetFunc();
        if(file.write(PTbuffer, halfbufPT)==-1) resetFunc();
        UpdateSID_REC(2,halfbufPT);  // update SID_REC with number of bytes written   
        time2writePT=0;
        if(LEDSON | introperiod) digitalWrite(LED_GRN,LOW);
      }
      if(time2writePT==2)
      {
        if(file.write(&SidRec[2],sizeof(SID_REC))==-1) resetFunc();
        if(file.write((const void*)&PTbuffer[halfbufPT], halfbufPT)==-1) resetFunc();
        UpdateSID_REC(2,halfbufPT);  // update SID_REC with number of bytes written       
        time2writePT=0;
      }    
              
      // check for short every two seconds and burn wire
      if(checkstop==1)
      {    
        checkstop = 0;
        updatetime(&nowtime);  //load into time structure
        nowtimesec=RTCToUNIXTime(&nowtime);     
        //Serial.println(nowtimesec); 
        
        calcPressTemp();

        if(depth < i2cpowdepth) {
          I2CPOW_STATE = HIGH;
        }
        else{
          I2CPOW_STATE = LOW;
        }

        if(burnflag>0)
        {
           ULONG burnduration = nowtimesec - burntimesec;
           if ((burnduration>0) & (burnduration<3600)){  //burn for 1 hour
             digitalWrite(BURN,HIGH);  //burn baby burn
           }
           else{
             digitalWrite(BURN,LOW);
           }
           if(burntimesec < nowtimesec) I2CPOW_STATE = HIGH; // VHF always on after burn
        }

        digitalWrite(I2CPOW, I2CPOW_STATE);

        if (abs(magnetom_x-old_magnetom_x)<inactivity_threshold) InactiveTime+=2;
        else InactiveTime=0;
        old_magnetom_x = magnetom_x;
        if (InactiveTime>inactivity_duration & alarmstatus==0 & motionflag) InactiveInt();  //if inactive and not woken by alarm
        
        if(alarmstatus==1) //woke via alarm
        {
          ULONG recduration = nowtimesec-waketimesec;
          if(recduration>recsec){ 
            waketime_hour+=sleep_hour;
            waketime_minute+=sleep_minute;
            if (waketime_minute > 59){
              waketime_minute -= 60;
              waketime_hour+=1;
            }
            if (waketime_hour > 23){
              waketime_hour-=24;
            }
            set_alarm(waketime_hour,waketime_minute);
            InactiveInt();  // recorded enough, go to sleep
          }
        }
        
        int stopval = analogRead(A2); //Stop pads shorted?
        if(stopval<20) 
        {
            FileStop();
            digitalWrite(LED_RED, HIGH);
            trueDelay(10000);
            digitalWrite(LED_RED, LOW);
            state = GO_TO_SLEEP;
        }
        
        // continuous record--new file every day or every hour
        if(alarmflag==0 & motionflag==0 & ((date!=lastdate & neweveryday) | (hour!=lasthour & neweveryhour)))
        {
          lastdate=date;
          lasthour=hour;
          introperiod = 0;
          file.timestamp(T_WRITE,(uint16_t) year+2000,month,date,hour,minute,second);
          file.close();
          FileInit();
        }
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
     if(time2write==1) //check for overflow--convert this to lighting LED
     { 
       overflow=1;
     // if(LEDSON | introperiod) digitalWrite(LED_RED,HIGH);
     }
     else
     {
       overflow=0;
     //   if(LEDSON | introperiod) digitalWrite(LED_RED,LOW);
      }
    time2write=2;  // set flag to write second half
    firstwritten=0; 
 }
 
  if((bufferpos>=halfbuf) & !firstwritten)  //at end of first buffer
  {
    if(time2write==2)
      {  
        overflow=1;
     //   if(LEDSON) digitalWrite(LED_RED,HIGH);
      }
    else
    {
        overflow=0;
     //   if(LEDSON) digitalWrite(LED_RED,LOW);
    }
    time2write=1; 
    firstwritten=1;  //flag to prevent first half from being written more than once; reset when reach end of double buffer
  }
}

// increment PTbuffer position by 1 byte. This does not check for overflow, because collected at a slow rate
void incrementPTbufpos(){
  boolean overflow;
  bufferposPT++;
   if(bufferposPT==PTBUFFERSIZE)
   {
     bufferposPT=0;
     time2writePT=2;  // set flag to write second half
     firstwrittenPT=0; 
   }
 
  if((bufferposPT>=halfbufPT) & !firstwrittenPT)  //at end of first buffer
  {
    time2writePT=1; 
    firstwrittenPT=1;  //flag to prevent first half from being written more than once; reset when reach end of double buffer
  }
}

/********************************************************************
***        Master Interrupt Routine to Read Sensors               ***
********************************************************************/
void flash(){  
  mscale_counter++;
  mscale_PTcounter++;
  RTCcounter++;

  if(RTCcounter>(2000.0/(unsigned int) speriod))  //reading here because interrupt blocks reads outside this loop sometimes
  {
    Read_RTC();
    RTCcounter=0;
    checkstop = 1;
  }

  if(mscale_PTcounter>=mscale_PTperiod)  //alternate reading temperature and pressure
  {
    updatepressflag = !updatepressflag;
    if(updatepressflag)
    {
      Read_Press();  //read current value of pressure and temperature
      PTbuffer[bufferposPT]=Pbuff[0];
      incrementPTbufpos();
      PTbuffer[bufferposPT]=Pbuff[1];
      incrementPTbufpos();
      PTbuffer[bufferposPT]=Pbuff[2];
      incrementPTbufpos();
      Update_Temp();
    }
    else
    {
        Read_Temp();
        PTbuffer[bufferposPT]=Tbuff[0];
        incrementPTbufpos();
        PTbuffer[bufferposPT]=Tbuff[1];
        incrementPTbufpos();
        PTbuffer[bufferposPT]=Tbuff[2];
        incrementPTbufpos();
        Update_Press();
    }
    mscale_PTcounter=0;
  }
 
    if(mscale_counter>=mscale_period)
    {
        //Write acceleromter values to buffer
        if(accelflagint)
        {      
         Read_Accel(AccelAddressInt);
         // Read_IMU(AccelAddressInt, 0x32, 6, 0);
          buffer[bufferpos]=(unsigned int)accel_x_int;  
          incrementbufpos();
          buffer[bufferpos]=(unsigned int)accel_x_int>>8;  
          incrementbufpos();
          buffer[bufferpos]=(unsigned int)accel_y_int;
          incrementbufpos();
          buffer[bufferpos]=(unsigned int)accel_y_int>>8;
          incrementbufpos(); 
          buffer[bufferpos]=(unsigned int)accel_z_int;
          incrementbufpos();
          buffer[bufferpos]=(unsigned int)accel_z_int>>8;
          incrementbufpos();
          
        }
        
        // Write Magnetometer to buffer
        if(compflag)
        {
          Read_Compass();
          buffer[bufferpos]=(unsigned int)magnetom_x;  
          incrementbufpos();
          buffer[bufferpos]=(unsigned int)magnetom_x>>8;  
          incrementbufpos();
          buffer[bufferpos]=(unsigned int)magnetom_y;
          incrementbufpos();
          buffer[bufferpos]=(unsigned int)magnetom_y>>8;
          incrementbufpos(); 
          buffer[bufferpos]=(unsigned int)magnetom_z;
          incrementbufpos();
          buffer[bufferpos]=(unsigned int)magnetom_z>>8;
          incrementbufpos();
        }
      
        // Write Gyros to buffer
        if(gyroflag)
        {
          //Read_IMU(GyroAddress, 0x1D, 6, 1);
          Read_Gyro();
          buffer[bufferpos]=(unsigned int)gyro_x;  
          incrementbufpos();
          buffer[bufferpos]=(unsigned int)gyro_x>>8;  
          incrementbufpos();
          buffer[bufferpos]=(unsigned int)gyro_y;
          incrementbufpos();
          buffer[bufferpos]=(unsigned int)gyro_y>>8;
          incrementbufpos(); 
          buffer[bufferpos]=(unsigned int)gyro_z;
          incrementbufpos();
          buffer[bufferpos]=(unsigned int)gyro_z>>8;
          incrementbufpos();
        }
      mscale_counter=0;
  }
}

void updatetime(TIME_HEAD *tm)
{ 
   	tm->sec=second;
	tm->minute=minute;  
	tm->hour=hour;  
	tm->day=1;  
	tm->mday=date;  
	tm->month=month;  
	tm->year=year;  
	tm->timezone=0;  
}

// Calculates Accurate UNIX Time Based on RTC Timestamp
unsigned long RTCToUNIXTime(TIME_HEAD *tm){
	int i;
	unsigned const char DaysInMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};
	unsigned long Ticks = 0;

	long yearsSince = tm->year+30; // Same as tm->year + 2000 - 1970
	long numLeaps = yearsSince >> 2; // yearsSince / 4 truncated
	
	if((!(tm->year%4)) && (tm->month>2)) Ticks+=SECONDS_IN_DAY;  //dm 8/9/2012  If current year is leap, add one day

	// Calculate Year Ticks
	Ticks += (yearsSince-numLeaps)*SECONDS_IN_YEAR;
	Ticks += numLeaps * SECONDS_IN_LEAP;

	// Calculate Month Ticks
	for(i=0; i < tm->month-1; i++){
	     Ticks += DaysInMonth[i] * SECONDS_IN_DAY;
	}

	// Calculate Day Ticks
	Ticks += tm->mday * SECONDS_IN_DAY;
	
	// Calculate Time Ticks CHANGES ARE HERE
	Ticks += (ULONG)tm->hour * SECONDS_IN_HOUR;
	Ticks += (ULONG)tm->minute * SECONDS_IN_MINUTE;
	Ticks += tm->sec;

	return Ticks;
}

int AddSid(int i, char* sid, unsigned long nbytes, unsigned long storetype, unsigned long dform, unsigned long SPus, unsigned long numchan, unsigned long sensors)
{
	unsigned long _sid, nelements;
	memcpy(&_sid, sid, 4);

	memset(&SidSpec[i], 0, sizeof(SID_SPEC));
        nbytes<<1;  //multiply by two because halfbuf

	switch(dform)
	{
		case DFORM_SHORT:
			nelements = nbytes>>1;
			break;
             
		case DFORM_LONG:
			nelements = nbytes/4;  //32 bit values
			break;
             
		case DFORM_I24:
			nelements = nbytes/3;  //24 bit values
			break;
	}

	SidSpec[i].SID = _sid;
	SidSpec[i].nBytes = nbytes;
	SidSpec[i].StoreType = storetype;
	SidSpec[i].DForm = dform;
	SidSpec[i].SPus = SPus*1000;  //convert from ms to us
	SidSpec[i].RECPTS = nelements;
	SidSpec[i].RECINT = nelements;
	SidSpec[i].NumChan = numchan;
        SidSpec[i].SensorType = sensors;	
	
	if(file.write(&SidSpec[i], sizeof(SID_SPEC))==-1)  resetFunc();
       	return i;
}

void UpdateSID_REC(int buf, int nbytes)
{
   unsigned long newcount=SidRec[buf].nbytes + nbytes; 
   if(newcount<SidRec[buf].nbytes) //check for rollover and use TS256_2 to count rollovers
       SidRec[buf].nbytes_2++;
   SidRec[buf].nbytes=newcount;  //this is total sample points from when recorder started (not reset to 0 when new files started)
}

void system_init(){
    pinMode(BURN, OUTPUT);      
    pinMode(I2CPOW, OUTPUT);  
    pinMode(LED_GRN, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(2, INPUT); //Arduino Interrupt2
    pinMode(3, INPUT); //Arduino Interrupt1
    pinMode(A2, INPUT);  //used to detect stop
    digitalWrite(A2, HIGH);
    digitalWrite(BURN,LOW);
    digitalWrite(LED_RED,LOW);
    digitalWrite(LED_GRN,HIGH);
    digitalWrite(I2CPOW, HIGH);
    analogReference(DEFAULT);
}

void SDcard_init(){
    pinMode(SD_POW, OUTPUT);      
    digitalWrite(SD_POW, HIGH);  //turn on power to SD card.  High for OpenTag.
    trueDelay(200);
    if (!sd.begin(CS, SPI_FULL_SPEED)) resetFunc();
}
void buffer_counter_reset(){
    mscale_counter=0;
    mscale_PTcounter=0;
    time2write=0;
    time2writePT=0;
    bufferpos=0;
    bufferposPT=0;
}

void sensor_init(){
    Gyro_Init(gyroflag,(float) 1000/iperiod);  //if gyroflag=0; will put to sleep if sensor present, (float) 1000/iperiod is srate
    Compass_Init(compflag); //if compflag=0; will put in Idle mode if sensor present
  
    if (pressflag|tempflag) 
    {
      Press_Init(); 
      Update_Press();
    }

    if (compflag)
    {
      Read_Compass();
    }
   
    if(accelflagint)
    {
         Accel_Init(AccelAddressInt,(float) 1000/iperiod,0, threshold);
         Read_Accel(AccelAddressInt);
    }
    accelint = Read_Accel_Int(AccelAddressInt); //read to clear accelerometer interrupts
    // set prescale to 8 for ADC to allow faster reads
    cbi(ADCSRA,ADPS2) ;
    sbi(ADCSRA,ADPS1) ;
    sbi(ADCSRA,ADPS0) ;
    
    //set system clock prescaler
    setClockPrescaler(clockprescaler); // set clockprescaler from script file
}

void system_start(){
   system_init();
   sensor_init();
   SDcard_init();
   FileInit();
   reset_alarm();
   buffer_counter_reset();
   StartInterruptTimer(speriod, clockprescaler);
}

void FileInit()
{
   // open file 
   count += 1;
   sprintf(filename,"%d.dsg",count);
  //Create a file. If the file already exists, increment counter and try again.
  while(!file.open(filename, O_CREAT | O_EXCL | O_WRITE))
  {
   count+=1; 
   sprintf(filename,"%d.dsg",count);
   if(count>1000000) resetFunc();
  // if(LEDSON | introperiod)  digitalWrite(LED_RED,HIGH);
  }
 
  Read_RTC();  //update time
  file.timestamp(T_CREATE,(uint16_t) year+2000,month,date,hour,minute,second);
  lastdate=date;  //store date when file created
  
  //Write file header
  int nSid, nSid0, nSid1, nSid2; 
  updatetime(&dfh.RecStartTime);
  file.write(&dfh, sizeof(DF_HEAD));
  nSid=0;  //reset Sid ID counter   
  if(accelflagint|gyroflag|compflag)
  {
        SidRec[0].nSID=AddSid(nSid,"INER", halfbuf, gyrorange, DFORM_SHORT, iperiod,(accelflagint*3)+(gyroflag*3)+(compflag*3),accelflagint<<5 | compflag<<4 | gyroflag<<3);
        SidRec[0].Chan = (accelflagint<<5 | compflag<<4 | gyroflag<<3);
        nSid++;  
  }

  if(tempflag|pressflag)
  {
        SidRec[2].nSID=AddSid(nSid,"PTMP", halfbufPT, EVTYPE_STREAM, DFORM_I24, PTperiod,(tempflag)+(pressflag), tempflag<<1 | pressflag<<2);
        SidRec[2].Chan = (tempflag<<1 | pressflag<<2);
        nSid++;  
  }

  AddSid(nSid, "DONE", 0, EVTYPE_STREAM, DFORM_SHORT, 1, 0, 0);  //last SID_SPEC with 0 for nbytes to indicate end of SID_SPEC headers
}

void FileStop(){
  introperiod = 0;
  StopTimer();
  file.timestamp(T_WRITE,(uint16_t) year+2000,month,date,hour,minute,second);
  file.close();
}

void StartInterruptTimer(int speriod, byte clockprescaler)
{
    MsTimer2::set(speriod>>clockprescaler, flash); // bitshift by clockprescaler...will round if not even
    MsTimer2::start();
}

void StopTimer()
{
    MsTimer2::stop();
}

void ActiveInt()
{
    //nothing here in case keeps firing...just need to wake up
}

void Alarm()
{
  //woke via alarm interrupt
  alarmstatus = 1;
}

void InactiveInt()
{
  if (state==RECORDING)
  {
    FileStop();
    InactiveTime = 0;
    alarmstatus = 0;
    accelint = Read_Accel_Int(AccelAddressInt); //read to clear accelerometer interrupts
    trueDelay(100);
    state = GO_TO_SLEEP;  
  }
}

//****************************************************************  
// set system into the sleep state 
// system wakes up when interrupt detected
void system_sleep() {
  // make all pin inputs and enable pullups to reduce power
  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  power_all_disable();
  if(alarmflag) attachInterrupt(0, Alarm, LOW); //RTC
  if(motionflag) attachInterrupt(1, ActiveInt, LOW); //acceleration activity interrupt
  sleep_mode();  // go to sleep
  // ...sleeping here....  
  sleep_disable();
  if(motionflag) detachInterrupt(1); //disable interrupt
  if(alarmflag) detachInterrupt(0);
  power_all_enable();
  sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON
}

void calcPressTemp(){
  float D1 = (float)((((unsigned long)Pbuff[0]<<16) | ((unsigned long)Pbuff[1]<<8) | ((unsigned long) Pbuff[2])));
  float D2 = (float)((((unsigned long)Tbuff[0]<<16) | ((unsigned long)Tbuff[1]<<8) | ((unsigned long) Tbuff[2])));
  
  float dT = D2 - ((float) TREF * 256.0);
  float T16 = (2000.0 + dT * (float)TEMPSENS / 8388608.0) / 100.0;
  
  float OFF = ((float) POFF * 65536.0)  + (((float) TCOFF * dT) / 128.0);
  float SENS = ((float) PSENS * 32768.0) + ((dT * (float) TCSENS) / 256.0);
  
  float P16 = (D1 * SENS / 2097152.0 - OFF) / 8192.0 / 10.0;  // mbar

  depth = -(1010.0 - P16) / 111.3; // 111.3=mBar/meter
}
