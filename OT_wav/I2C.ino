/* ******************************************************* */
/* I2C code for external sensors                           */
/* ******************************************************* */

void Accel_Init(int AccelAddress, float srate, boolean lowpower, byte threshold)
{
  Wire.beginTransmission(AccelAddress);
  Wire.write(0x2D);  // power register
  Wire.write(0x0B);  // measurement mode
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(AccelAddress);
  Wire.write(0x31);  // Data format register
  // Wire.write(0x08);  // set to 2g
  // Wire.write(0x0F); //Full_resolution mode, left-justified (MSB), +/-16g
  Wire.write(0x2B); //Full_resolution mode, sign bit, +/-16g, INT_LOW
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
  Wire.write(0x2C);  // Rate
  // set to next higher sample rate on compass
 int ratecode=8;  //default to 25 Hz
 if (srate>25) ratecode=9;
 if (srate>50) ratecode=10;
 if (srate>100) ratecode=11;
 if (srate>200) ratecode=12;
 if (srate>400) ratecode=13;
 if (srate>800) ratecode=14;
 if (srate>1600) ratecode=15;
 ratecode = ratecode | (lowpower<<4);
  Wire.write(ratecode );  // set to 800Hz, normal operation
  Wire.endTransmission();
  delay(5);
  
  // motion interrupt setup
  Wire.beginTransmission(AccelAddress);
  Wire.write(0x24);  // Activity register
  Wire.write(threshold);  // Activity threshold set to 62.5 mg/lsb 16=0x10=1g
  Wire.write(0x10);  // Inactivity threshold
  Wire.write(0x3C);  // Inactivity duration 0x3C=60s'
  Wire.write(0xFF);  // ACT_INACT_CTL D7=1 AC-coupled; D0-D6: All axes enabled
  Wire.endTransmission();
  delay(5);
  
  Wire.beginTransmission(AccelAddress);
  Wire.write(0x2F);  // INT_MAP register
  Wire.write(0x08);  // INT_MAP D4=Activity=0 = INT1, D3=Inactivity=1=INT2
  Wire.endTransmission();
  delay(5);
  
  // motion interrupt enable
  Wire.beginTransmission(AccelAddress);
  Wire.write(0x2e);  // INT_ENABLE address
  Wire.write(0x18);  // INT_ENABLE D4=Activity, D3=Inactivity
  Wire.endTransmission();
  delay(5);
}

int readAccelInt(int AccelAddress)  //read accelerometer interrupts; used to clear`
{
  byte buff;
  Wire.beginTransmission(AccelAddress); 
  Wire.write(0x30);        //sends address to read from
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(AccelAddress, 6);    // request 6 bytes from device
  buff = Wire.read();  // receive one byte
  return buff;
}

// Reads z accelerometer register
void Read_Accel(int AccelAddress)
{
  int i = 0;
  Wire.beginTransmission(AccelAddress); 
  Wire.write(0x36);        //sends address to read from
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(AccelAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  {    
     buffer[bufferpos] = Wire.read();  // receive one byte
     incrementbufpos();
  }
}


 
