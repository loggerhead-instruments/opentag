# otload.R: Read in OpenTag .DSG files
# Converts raw values to calibrated units
# Assumes all sensors are recorded (accel, mag, gyro, pressure, temperature)

# To Do:
# - estimate size of data to be read
# - preallocate vectors

# Required packages
require(lubridate)

# load multiple opentag files and merge together
# specify path for folder of files to load
path = "/w/loggerhead/opentag/R/sample/R012/"

startfile = 2  # specify name of file to start with
endfile = 10    # specify name of file to end with

# initialize empty vectors based on size of first file
numfiles = endfile - startfile + 1
filename = paste(path, startfile, ".DSG", sep="") 
samples = file.size(filename) / 2 # this is overestimate
INER <- rep(NA,samples * numfiles)  # Inertial vector
PTMP <- rep(NA,samples * numfiles)   # Pressure/Temperature vector

# Calibration constants for pressure/temperature
# these are uints
calfilename = paste(path, "PRESSTMP.CAL", sep ="")
calfile = file(calfilename, "rb")
PSENS = readBin(calfile, integer(), n=1, size=2, signed = FALSE, endian = "little");
POFF = readBin(calfile, integer(), n=1, size=2, signed = FALSE, endian = "little");
TCSENS = readBin(calfile, integer(), n=1, size=2, signed = FALSE, endian = "little");
TCOFF = readBin(calfile, integer(), n=1, size=2, signed = FALSE, endian = "little");
TREF = readBin(calfile, integer(), n=1, size=2, signed = FALSE, endian = "little");
TEMPSENS = readBin(calfile, integer(), n=1, size=2, signed = FALSE, endian = "little");
close(calfile)

startIMU = 1
endIMU = 1
startPTMP = 1
endPTMP = 1

for (filenum in startfile:endfile){
  filename = paste(path, filenum, ".DSG", sep="")
  print(Sys.time())
  print(filename)
  datafile = file(filename, "rb")

  # DF_HEAD
  version = readBin(datafile, integer(), n = 1, size = 4, endian = "little")
  userID = readBin(datafile, integer(), n = 1, size = 4, endian = "little")
  second = readBin(datafile, integer(), n = 1, size = 1, endian = "little")
  minute = readBin(datafile, integer(), n = 1, size = 1, endian = "little")
  hour = readBin(datafile, integer(), n = 1, size = 1, endian = "little")
  day = readBin(datafile, integer(), n = 1, size = 1, endian = "little")
  mday = readBin(datafile, integer(), n = 1, size = 1, endian = "little")
  month = readBin(datafile, integer(), n = 1, size = 1, endian = "little")
  year = readBin(datafile, integer(), n = 1, size = 1, endian = "little")
  timezone = readBin(datafile, integer(), n = 1, size = 1, endian = "little")
  
  if (version >= 1010){
    lat = readBin(datafile, numeric(), n = 1, size = 4, endian = "little")
    lon = readBin(datafile, numeric(), n = 1, size = 4, endian = "little")
    depth = readBin(datafile, numeric(), n = 1, size = 4, endian = "little")
    DSGcal = readBin(datafile, numeric(), n = 1, size = 4, endian = "little")
    hydroCal = readBin(datafile, numeric(), n = 1, size = 4, endian = "little")
    lpFilt = readBin(datafile, numeric(), n = 1, size = 4, endian = "little")
  }
  
  # SID_SPEC
  notdone = 1
  nSIDSPEC = 1
  SID <- vector()
  nBytes <- vector()
  numChan <- vector()
  storeType <- vector()
  sensorType <- vector()
  dForm <- vector()
  period <- vector()
  recpts <- vector()
  recint <- vector()
  
  while (notdone == 1){
    SID[nSIDSPEC] = intToUtf8(readBin(datafile, integer(), n = 4, size = 1, endian = "little"))
    nBytes[nSIDSPEC] = readBin(datafile, integer(), n = 1, size = 4, endian = "little")
    numChan[nSIDSPEC] = readBin(datafile, integer(), n = 1, size = 4, endian = "little")
    storeType[nSIDSPEC] = readBin(datafile, integer(), n = 1, size = 4, endian = "little")
    sensorType[nSIDSPEC] = readBin(datafile, integer(), n = 1, size = 4, endian = "little")
    dForm[nSIDSPEC] = readBin(datafile, integer(), n = 1, size = 4, endian = "little")
    period[nSIDSPEC] = readBin(datafile, integer(), n = 1, size = 4, endian = "little")
    recpts[nSIDSPEC] = readBin(datafile, integer(), n = 1, size = 4, endian = "little")
    recint[nSIDSPEC] = readBin(datafile, integer(), n = 1, size = 4, endian = "little")
    
    if(nBytes[nSIDSPEC] == 0){
      notdone = 0
    }
    nSIDSPEC = nSIDSPEC + 1
  }
  nSIDSPEC = nSIDSPEC - 2
  
  # SID_REC 
  # IMU dataframe
  while (1) {
      nSID = readBin(datafile, integer(), n = 1, size = 1, endian = "little")
      chan = readBin(datafile, integer(), n = 1, size = 1, endian = "little")
      nbytes = readBin(datafile, integer(), n = 1, size = 4, endian = "little")
      if (version > 9999){
        nbytes_2 = readBin(datafile, integer(), n = 1, size = 4, endian = "little")
      }
      cur_sid = nSID + 1
      if (length(cur_sid) == 0){
        break;
      }
      if(cur_sid>0 & cur_sid<8){
        if(dForm[cur_sid] == 2){
          nsamples = nBytes[cur_sid] / 2
          chunk = readBin(datafile, integer(), n = nsamples, size = 2, endian = "little")
        }
        if(dForm[cur_sid] == 3){
          nsamples = nBytes[cur_sid] / 3
          chunk = readBin(datafile, integer(), n = nsamples * 3, signed = FALSE, size = 1, endian = "little")
        }
        if(dForm[cur_sid] == 4){
          nsamples = nBytes[cur_sid] / 4
          chunk = readBin(datafile, numeric(), n = nsamples, size = 4, endian = "little")
        }
        # add to appropriate dataframe as read in
        
        if(SID[cur_sid] == "INER"){
          #dim(chunk) <- c(length(chunk) / numChan[cur_sid], numChan[cur_sid]) ## (rows, cols)
          #INER_df9 = rbind(INER_df9, data.frame(chunk))
          
          endIMU = startIMU + length(chunk) - 1
          INER[startIMU:endIMU] <- chunk
          startIMU = endIMU + 1
        }
        if(SID[cur_sid] == "PTMP"){
          #dim(chunk) <- c(6,length(chunk) /  6, 6) ## (rows, cols)
          #PTMP_df8 = rbind(PTMP_df8, data.frame(chunk))
          endPTMP = startPTMP + length(chunk) - 1
          PTMP[startPTMP:endPTMP] <- chunk
          startPTMP = endPTMP + 1
        }
      }
  }
  close(datafile)
}

# calibrate values
srate=1000000.0/(period[1]);
accel_cal=16.0/4096.0;  #16 g/4096 (13 bit ADC)
gyro_cal=500.0/32768.0;  # 500 degrees per second (16-bit ADC)
mag_cal=1.0/1090.0;  #1090 LSB/Gauss

# Inertial headings calibration
n = endIMU
INER = data.frame("accelX" = accel_cal * INER[seq(1, n, 9)],
                  "accelY" = accel_cal * INER[seq(2, n, 9)],
                  "accelZ" = accel_cal * INER[seq(3, n, 9)],
                  "magX" = mag_cal * INER[seq(4, n, 9)],
                  "magY" = mag_cal * INER[seq(5, n, 9)],
                  "magZ" = mag_cal * INER[seq(6, n, 9)],
                  "gyroX" = gyro_cal * INER[seq(7, n, 9)],
                  "gyroY" = gyro_cal * INER[seq(8, n, 9)],
                  "gyroZ" = gyro_cal * INER[seq(9, n, 9)]
)

# Pressure/Temperature
# Combine values into 24-bit value and use calibration constants from files

n = endPTMP
D1 = (PTMP[seq(1, n, 6)] * 65536.0) + (PTMP[seq(2, n, 6)] * 256.0) + PTMP[seq(3, n, 6)]
D2 = (PTMP[seq(4, n, 6)] * 65536.0) + (PTMP[seq(5, n, 6)] * 256.0) + PTMP[seq(6, n, 6)]
dT = D2 - TREF * 256.0
OFF = POFF * 65536.0 + (TCOFF * dT) / 128.0
SENS = PSENS * 32768.0 + (dT * TCSENS) / 256.0

# mbar (i.e. a value of 1 = 1 mbar = ~1 cm depth resolution) 
PTMP = data.frame("temperature" = (2000.0 + dT * TEMPSENS / 8388608.0) / 100.0 ,
                  "pressure" = (D1 * SENS / 2097152.0 - OFF) / 81920.0)

# Datetime
startDT = make_datetime(year = year + 2000, month=month, day=mday, hour=hour, min=minute, sec=second, tz="UTC")
periodS = period / 1000000.0
# Pressure/Temp
n = nrow(PTMP)
duration = n * periodS[2]  #duration in seconds
endDT = startDT + dseconds(duration)
PTMP$datetime = seq(startDT, endDT, length.out = n)

# Inertial
n = nrow(INER)
duration = n * periodS[1] / 1000000.0 #duration in seconds
endDT = startDT + dseconds(duration)
INER$datetime = seq(startDT, endDT, length.out = n)