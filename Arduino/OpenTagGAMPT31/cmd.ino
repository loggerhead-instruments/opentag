#define CMD(a,b) ( a + (b << 8))
#define TRUE 1
#define FALSE 0

int ProcCmd(char *pCmd)
{
	short *pCV;
	short n;
	unsigned int lv1;
	char s[22];
        unsigned int tday;
        unsigned int tmonth;
        unsigned int tyear;
        unsigned int thour;
        unsigned int tmin;
        unsigned int tsec;
        byte rec_hour;
        byte rec_min;

	pCV = (short*)pCmd;

	n = strlen(pCmd);
	if(n<2) 
          return TRUE;

	switch(*pCV)
	{              
                case ('V' + ('1'<<8)):  
                {
                   sscanf(&pCmd[3],"%d",&lv1);
                   speriod=lv1/1000;  // Master interrupt rate in ms. 
                   break;
                }
                
                case ('C' + ('P'<<8)):
                {
                   sscanf(&pCmd[3],"%d",&lv1);
                   clockprescaler=lv1;
                   break; 
                }
  
                //SC is IMU sample period in ms
                case ('S' + ('C'<<8)):
                {
                   sscanf(&pCmd[3],"%d",&lv1);
                   iperiod=lv1;
                   break; 
                  
                }
		// Sets real time clock
		case ('T' + ('M'<<8)):
		{
                   //set time
                   sscanf(&pCmd[3],"%d-%d-%d %d:%d:%d",&tmonth,&tday,&tyear,&thour,&tmin,&tsec);
                   
                   // if earlier than current time...do not reset
                   TIME_HEAD NewTime;
                   second=tsec;
                   minute=tmin;
                   hour=thour;
                   date=tday;
                   month=tmonth;
                   year=tyear;
                   updatetime(&NewTime);  //load into structure
                   ULONG newtime=RTCToUNIXTime(&NewTime);  //get new time in seconds
                   Read_RTC();  //update time with current clock time
                   updatetime(&dfh.RecStartTime);  //load current clock time into DF_HEAD time structure
                   ULONG curtime=RTCToUNIXTime(&dfh.RecStartTime);
                   
                   if(newtime>curtime)
                     setTime2(thour,tmin,tsec,tday,tmonth,tyear);    
     		  break;	
		}

                // Force set of Real Time Clock
		case ('F' + ('T'<<8)):
		{
                   //set time
                   sscanf(&pCmd[3],"%d-%d-%d %d:%d:%d",&tmonth,&tday,&tyear,&thour,&tmin,&tsec);
                   setTime2(thour,tmin,tsec,tday,tmonth,tyear); 
                   break;
                }
                
                 // Burn Time
		case ('B' + ('W'<<8)):
		{
                   //get time
                   sscanf(&pCmd[3],"%d-%d-%d %d:%d:%d",&tmonth,&tday,&tyear,&thour,&tmin,&tsec);
                   burntime.sec=tsec;
                   burntime.minute=tmin;
                   burntime.hour=thour;
                   burntime.mday=tday;
                   burntime.month=tmonth;
                   burntime.year=tyear;
                   burnflag=1;
                   burntimesec=RTCToUNIXTime(&burntime);
                   break;
                }               
                
                // Burn Timer
		case ('B' + ('T'<<8)):
		{
                   sscanf(&pCmd[3],"%d:%d",&thour, &tmin);
                   burnInSeconds = ((thour * 60) + tmin) * 60;
                   burnflag=2;
                   break;
                } 
                
                // I2CPOW toggle based on interval and depth
		case ('I' + ('S'<<8)):
		{
                   sscanf(&pCmd[3],"%d %d",&i2cpowinterval, &i2cpowdepth);
                   break;
                } 
                
                // Wake Time
		case ('W' + ('T'<<8)):
		{
                   //get time
                   sscanf(&pCmd[3],"%d:%d %d:%d %d:%d", &thour, &tmin, &rec_hour, &rec_min, &sleep_hour, &sleep_minute);
                   waketime_minute=tmin;
                   waketime_hour=thour;
                   recsec = (rec_hour*3600) + (rec_min*60);
                   alarmflag=1;
                   set_alarm(thour,tmin);
                   break;
                }  
                
                case ('M' + ('W'<<8)):
		{
		    motionflag = 1;
                    sscanf(&pCmd[3],"%d %d %d",&lv1, &tmin, &tsec);
                    threshold = (byte)((float)lv1/62.5); // Activity threshold set to 62.5 mg/lsb 16=0x10=1g
                    inactivity_threshold = tmin;
                    inactivity_duration = tsec;
                    break;
		}

		// Disable LEDS
		case ('L' + ('D'<<8)):
		{
		    LEDSON=0;
		    break;
		}

                // New file every hour. Default is new file every hour
                case ('N' + ('D'<<8)):
                {
                    neweveryday = 1; 
                    neweveryhour = 0;
                }
		// Board Accelerometer
		case ('A' + ('I'<<8)):
		{
		    accelflagint=1;
		    break;
		}
		// Magnetometer
		case ('M' + ('G'<<8)):
		{
		    compflag=1;
		    break;
		}
		// Gyroscope
		case ('G' + ('Y'<<8)):
		{
		    sscanf(&pCmd[3],"%d",&lv1);
                    gyroflag=1;
                    // 0 +/- 250 degrees/s
                    // 1 +/- 500 degrees/s
                    // 2 +/- 1000 degrees/s
                    // 3 +/- 2000 degrees/s
                    gyrorange=lv1;
		    break;
		}
		// PT Period
		case ('P' + ('T'<<8)):
		{
		     sscanf(&pCmd[3],"%d",&lv1);
                     PTperiod=lv1;  // Pressure/temperature period in ms
		    break;
		}
		// Temperature
		case ('T' + ('P'<<8)):
		{
		   tempflag=1;
		    break;
		}
		// Pressure
		case ('P' + ('R'<<8)):
		{
		   pressflag=1;
		   break;
		}
	}	
	return TRUE;
}

void LoadScript(SdFile file)
{
  char s[22];
  char c;
  short i;
  int16_t n;

   // Read card setup.txt file to set date and time, sample rate, recording interval
   sd.chdir("Script");
   if(file.open("Default.txt", O_READ))
   {
      do{
	  i = 0;
	  s[i] = 0;
	  do{
               n=file.read(&c, 1);
	       if(c!='\r') s[i++] = c;
	       if(i>20) break;
	  }while(c!='\n');
	  s[--i] = 0;
          if(s[0] != '/' && i>1)
          {
              ProcCmd(s);
	  }
	}while(n);
	file.close();  
  }
  else
    {   
    //enable defaults if no script present
    accelflagint=1;  //flag to enable accelerometer; 
    compflag=1; //flag to enable 3d magnetometer (compass)
    gyroflag=1; //flag to enable gyro
    pressflag=1; //flag to enable pressure
    tempflag=1; //flag to enable temperature
 }

 sd.chdir();  //change to root
 return;	
}



