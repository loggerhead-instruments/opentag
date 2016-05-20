/* ******************************************************* */
/* I2C code for external sensors                           */
/* ******************************************************* */
int CompassAddress = 0x1E;  //0x3C //0x3D;  //(0x42>>1);
int GyroAddress = 0x69;
int RTCAddress = 0x68;
int PressAddress = 0x77;

void I2C_Init()
{
 Wire.begin();
}

void Press_Init()
{
  int i = 0;
  byte buff[2];
  int bytesread;
  
  // Reset so PROM is loaded
  Wire.beginTransmission(PressAddress);
  Wire.send(0x1E);  //Reset Command
  bytesread=Wire.endTransmission();
  //Serial.println("Pressure Reset 0=success");
  //Serial.println(bytesread);  
    
  delay(5);  //reset needs at least 2.8 ms
  
  // Read and store calibration coefficients
  Wire.beginTransmission(PressAddress);
  Wire.send(0xA2);  //PROM Read Pressure Sensitivity
  Wire.endTransmission();
  
  bytesread=Wire.requestFrom(PressAddress, 2);    // request 2 bytes from device
//  Serial.println("PROM Bytes Available");
//  Serial.println(bytesread);
  
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  PSENS=(unsigned int) (buff[0]<<8)|buff[1]; //pressure sensitivity  MSB first
    
  i=0;
  Wire.beginTransmission(PressAddress);
  Wire.send(0xA4);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(PressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  POFF=(buff[0]<<8)|buff[1];  //Pressure offset
 
  i=0;
  Wire.beginTransmission(PressAddress);
  Wire.send(0xA6);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(PressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  TCSENS=(buff[0]<<8)|buff[1];  //

  i=0;
  Wire.beginTransmission(PressAddress);
  Wire.send(0xA8);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(PressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }

  TCOFF=(buff[0]<<8)|buff[1];  //Temp coefficient of pressure offset
  
  i=0;
  Wire.beginTransmission(PressAddress);
  Wire.send(0xAA);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(PressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }

  TREF=(buff[0]<<8)|buff[1];  //Ref temperature
  
  i=0;
  Wire.beginTransmission(PressAddress);
  Wire.send(0xAC);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(PressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }

  TEMPSENS =(buff[0]<<8)|buff[1];  //Temperature sensitivity coefficient  

}

void Update_Press()
{
    Wire.beginTransmission(PressAddress);
    Wire.send(0x48);  //Initiate pressure conversion OSR-4096
    Wire.endTransmission();

}

void Update_Temp()
{
   Wire.beginTransmission(PressAddress);
   Wire.send(0x58); //Initiate Temperature conversion OSR=4096
   Wire.endTransmission();
}

void Read_Press()
{
  int i = 0;
  
  Wire.beginTransmission(PressAddress);
  Wire.send(0x00);  //ADC Read Command
  Wire.endTransmission();
  
  Wire.requestFrom(PressAddress, 3);    // request 3 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    Pbuff[i] = Wire.receive();  // receive one byte
    i++;
  }
  
  
}


void Read_Temp()
{
  int i = 0;
 
  Wire.beginTransmission(PressAddress);
  Wire.send(0x00);  //ADC Read Command
  Wire.endTransmission();
  
  Wire.requestFrom(PressAddress, 3);    // request 3 bytes from device
  i=0;
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    Tbuff[i] = Wire.receive();  // receive one byte
    i++;
  }

}

void Accel_Init(int AccelAddress, float srate, boolean lowpower, byte threshold)
{
 Wire.beginTransmission(AccelAddress);
  Wire.send(0x2D);  // power register
  Wire.send(0x0B);  // measurement mode
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(AccelAddress);
  Wire.send(0x31);  // Data format register
 // Wire.send(0x08);  // set to 2g
 // Wire.send(0x0F); //Full_resolution mode, left-justified (MSB), +/-16g
 Wire.send(0x2B); //Full_resolution mode, sign bit, +/-16g, INT_LOW
  Wire.endTransmission();
  delay(5);	

// Rate codes
// 800 Hz: 1101 0x0D 13
// 400 Hz: 1100 0x0C 12
// 200 Hz: 1011 0x0B 11
// 100 Hz: 1010 0x0A 10
// 50 Hz: 1001  0x09  9
// 25 Hz: 1000  0x08  8

// Bit 5 low power mode (lower power, but higher noise) 0=normal
  Wire.beginTransmission(AccelAddress);
  Wire.send(0x2C);  // Rate
  // set to next higher sample rate on compass
 int ratecode=8;  //default to 25 Hz
 if (srate>25) ratecode=9;
 if (srate>50) ratecode=10;
 if (srate>100) ratecode=11;
 if (srate>200) ratecode=12;
 if (srate>400) ratecode=13;
 ratecode = ratecode | (lowpower<<4);
  Wire.send(ratecode );  // set to 800Hz, normal operation
  Wire.endTransmission();
  delay(5);
  
  // motion interrupt setup
  Wire.beginTransmission(AccelAddress);
  Wire.send(0x24);  // Activity register
  Wire.send(threshold);  // Activity threshold set to 62.5 mg/lsb 16=0x10=1g
  Wire.send(0x10);  // Inactivity threshold
  Wire.send(0x3C);  // Inactivity duration 0x3C=60s'
  Wire.send(0xFF);  // ACT_INACT_CTL D7=1 AC-coupled; D0-D6: All axes enabled
  Wire.endTransmission();
  delay(5);
  
  Wire.beginTransmission(AccelAddress);
  Wire.send(0x2F);  // INT_MAP register
  Wire.send(0x08);  // INT_MAP D4=Activity=0 = INT1, D3=Inactivity=1=INT2
  Wire.endTransmission();
  
  // motion interrupt enable
  Wire.beginTransmission(AccelAddress);
  Wire.send(0x2e);  // INT_ENABLE address
  Wire.send(0x18);  // INT_ENABLE D4=Activity, D3=Inactivity
  Wire.endTransmission();
  delay(5);
}

int Read_Accel_Int(int AccelAddress)  //read accelerometer interrupts; used to clear`
{
  byte buff;
  Wire.beginTransmission(AccelAddress); 
  Wire.send(0x30);        //sends address to read from
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(AccelAddress, 6);    // request 6 bytes from device
  buff = Wire.receive();  // receive one byte
  return buff;
}

// Reads x,y and z accelerometer registers
void Read_Accel(int AccelAddress)
{
 int i = 0;
 byte buff[6];
  
  Wire.beginTransmission(AccelAddress); 
  Wire.send(0x32);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
//  Wire.beginTransmission(AccelAddress); //start transmission to device
  Wire.requestFrom(AccelAddress, 6);    // request 6 bytes from device
  //if(AccelAddress==AccelAddressInt)
 // {
    while(Wire.available())   // ((Wire.available())&&(i<6))
    { 
      //  buffer[bufferpos]=Wire.receive();  
      //  incrementbufpos();
     
       buff[i] = Wire.receive();  // receive one byte
       i++;
    }

  if (i==6)  // All bytes received?
    {
        accel_x_int = (((int)buff[1]) << 8) | buff[0];    // X axis (internal sensor x axis)
        accel_y_int = (((int)buff[3]) << 8) | buff[2];    // Y axis (internal sensor y axis)
        accel_z_int = (((int)buff[5]) << 8) | buff[4];    // Z axis
    }
   
  //else
   // Serial.println("ERR:Acc");
}

void Compass_Init(boolean mode)
{
  
  // default Sensor Range +/- 1.3 Ga.  LSB/Gauss = 1090
  
  Wire.beginTransmission(CompassAddress);
  // Configuration Register A (Register address 0x00)
  // Bit 7: 0
  // Bit 6-5: Samples Averaged b00=1; b01=2; b10=4; b11=8 (default)
  // Bit 4-2: Data Output Rate Bits b100=15 Hz (default); b101=30 Hz; b110=75 Hz.  
  //          Higher output rate (160 Hz) can be achieved by checking DRDY interrupt
  // Bit 1-0: Measurement configuration
  //          b00=Normal (default); b01=Positive bias for cal (use 0x11); b10=Negative bias for cal (use 0x12)
  Wire.send(0x00);  //Register address 0
  Wire.send(0x78);   // average 8 samples, 75 Hz rate, no bias


 // Configuration Register B (Register address 0x01)
 // Set Gain
  Wire.send(0x20); //set to +/- 1.3 Ga (1090 LSB/Gauss)

  // Set Measurement Mode
  // Register address 0x02
  // 0x00 : Continuous measurement mode (~100 uA draw)
  // 0x01 : Single Measurement mode
  // 0x02 : Idle Mode (2 uA draw)ot
  //Wire.send(0x00);   // Set continuous mode (default to 15Hz)
  if(mode==0)
      Wire.send(0x03); //Idle Mode
  else
    Wire.send(0x01);   // Single measurement mode (goes idle between measurements)
  Wire.endTransmission(); //end transmission
 
}
  
void Read_Compass()
{
 // HMC5883L stores X, Z, Y
 // Compass data will be -4096 if there is an overflow
  int i = 0;
  byte buff[6];
  
  Wire.beginTransmission(CompassAddress); 
  Wire.send(0x03);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
 // Wire.beginTransmission(CompassAddress); 
  Wire.requestFrom(CompassAddress, 6);    // request 6 bytes from device

  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    //  buffer[bufferpos]=Wire.receive();  
     // incrementbufpos();
    
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
 // Wire.endTransmission(); //end transmission


  if (i==6)  // All bytes received?
    {
    // MSB byte first, then LSB, X,Z,Y (order of reads from compass)
    magnetom_z = ((((int)buff[2]) << 8) | buff[3]);    
    magnetom_x = ((((int)buff[0]) << 8) | buff[1]);    
    magnetom_y = ((((int)buff[4]) << 8) | buff[5]);    
    }

 // else
  //  Serial.println("ERR:Mag");
  
  // in single measurement mode...make new measurement now
  Wire.beginTransmission(CompassAddress); 
  Wire.send(0x02);        //address of Mode Register
  
  Wire.send(0x01);   // Single measurement mode (goes idle between measurements)
  Wire.endTransmission(); //end transmission
   
}

void Gyro_Init(boolean mode, float srate)
{
  int ecode;
 //  Serial.print("Gyro Init\n");
   if(mode==0)
  {
     Wire.beginTransmission(GyroAddress);
     Wire.send(0x3E); //power management
     Wire.send(0x40); //sleep mode
     Wire.endTransmission(); //end transmission
     return;
  }
  
  Wire.beginTransmission(GyroAddress);
  Wire.send(0x3E); //power management
  Wire.send(0x01); //everything awake; clock from X gyro reference
  Wire.endTransmission(); //end transmission
  delay(5);
  

 //  Serial.print(ecode);
 //  Serial.print("\n");

// 
// bits 3-4: 
// 0 +/- 250 degrees/s
// 1 +/- 500 degrees/s
// 2 +/- 1000 degrees/s
// 3 +/- 2000 degrees/s

// Bits 0-2. Low pass filter setting
// 0 256 Hz
// 1 188 Hz
// 2 98 Hz
// 3 42 Hz
// 4 20 Hz
// 5 10 Hz
// 6 5 Hz
 
 int ratecode=6;  //default to 5 Hz
 if (srate>20) ratecode=5;  //  low pass 10 Hz
 if (srate>40) ratecode=4;  // low pass 20 Hz
 if (srate>80) ratecode=3;  //low-pass 42 Hz
 if (srate>160) ratecode=2; // low pass 98 Hz
 if (srate>360) ratecode=1;  // low pass 188 Hz

 ratecode = (ratecode | (gyrorange<<3));  // 1<<3 = 500 degree/s

  Wire.beginTransmission(GyroAddress);  
  Wire.send(0x16);  // gyro scale, sample rate and LPF
  Wire.send(ratecode);  
  ecode=Wire.endTransmission(); //end transmission
  delay(5);
 //  Serial.print("Gyro Setup; 0=good\n");
 //  Serial.print(ecode);
 //  Serial.print("\n");
 
  //Wire.beginTransmission(GyroAddress); 
  //Wire.send(0x3D);        //sends address to write
  //Wire.send(0x01);  
 // ecode=Wire.endTransmission(); //end transmission
}


void Read_Gyro()
{
  int i = 0;
  byte buff[6];
 // int ecode;
  int numbytestoread=6;
 
  Wire.beginTransmission(GyroAddress); 
  Wire.send(0x1D);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  //Wire.beginTransmission(GyroAddress); 
  Wire.requestFrom(GyroAddress, numbytestoread);    // request 6 bytes from device
  
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
   //  buffer[bufferpos]=Wire.receive();  
   //  incrementbufpos();
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  //Wire.endTransmission(); //end transmission


  if (i==numbytestoread)  // All bytes received?
    {
    // MSB byte first, then LSB, X,Y,Z
   // gyro_temp = ((((int)buff[0]) << 8) | buff[1]); 
    gyro_x = ((((int)buff[0]) << 8) | buff[1]);    // X axis (roll to right)
    gyro_y = ((((int)buff[2]) << 8) | buff[3]);    // Y axis (pitch forward)
    gyro_z = -1*((((int)buff[4]) << 8) | buff[5]);    // Z axis
    }

 // else
 //   Serial.println("ERR:Gyro");
}

void setTime2(int thour,int tminute,int tsecond,int tday,int tmonth,int tyear)
{
  Wire.beginTransmission(RTCAddress);
  Wire.send(0x00);  //starting address is 00
  Wire.send(((tsecond/10)<<4) | ((tsecond%10)&0x0F));  //seconds, 
  Wire.send(((tminute/10)<<4) | ((tminute%10)&0x0F));  //min
  Wire.send((((thour/10)&0x03)<<4) | ((thour)%10&0x0F)); //hour  0x20 will set to 12 hour AM/PM mode
  Wire.send(0x01); // day of week(1-7)
  Wire.send(((tday/10)<<4) | ((tday%10)&0x0F)); // date (1-31)
  Wire.send((tmonth/10)<<4 | ((tmonth%10)&0x0F)); //  month
  Wire.send((tyear/10)<<4 | ((tyear%10)&0x0F)); //  year 2010=0x0A
  int ecode=Wire.endTransmission(); //end transmission
}

void set_alarm(int thour, int tminute)
{
  Wire.beginTransmission(RTCAddress);
  Wire.send(0x07); //0x0B Alarm1
  Wire.send(0x00); //seconds
  Wire.send(((tminute/10)<<4) | ((tminute%10)&0x0F)); // Minutes
  Wire.send((((thour/10)&0x03)<<4) | ((thour)%10&0x0F)); //hour
  Wire.send(0x80); //alarm when hour minutes and seconds match
  Wire.endTransmission();

  Wire.beginTransmission(RTCAddress);
  Wire.send(0x0E); //Control register
  Wire.send(0x01); //enable Alarm 1 interrupt
  Wire.send(0x00); //clear existing interrupts
  Wire.endTransmission();
}

void reset_alarm()
{
  Wire.beginTransmission(RTCAddress);
  Wire.send(0x0F); //Control register
  Wire.send(0x00); //clear existing interrupts
  Wire.endTransmission();
}
void Read_RTC()
{
  int i = 0;
  byte buff[7];
 
  Wire.beginTransmission(RTCAddress); 
  Wire.send(0x00);        //sends address to read from starting with seconds
  Wire.endTransmission(); //end transmission
  
  Wire.requestFrom(RTCAddress, 7);    // request 7 bytes from clock (seconds through Year)
  int bytesavail=Wire.available();

  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  
  if (i==7)  // All bytes received?
  {
      second=(10*(buff[0]>>4)) + ((buff[0]&0x0F)) ;//lower 4 bits is seconds, upper 3 bits are 10 seconds
      minute=(10*(buff[1]>>4)) + ((buff[1]&0x0F)) ;//lower 4 bits is minutes, upper 3 bits are 10 minutes
      hour=(10*((buff[2]>>4)&0x03))+((buff[2]&0x0F));  //bits 4 and 5 are 10 hours; bit 0-3 are hour
      date=(10*(buff[4]>>4))+((buff[4]&0x0F)); 
      month=(10*(buff[5]>>4))+((buff[5]&0x0F)); 
      year=(10*(buff[6]>>4))+((buff[6]&0x0F));   
  }
}
 
 
