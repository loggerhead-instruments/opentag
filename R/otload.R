# otload.R: Read in OpenTag .DSG files
# Converts raw values to calibrated units

# To do:
# - check values correct
# - add datetime

filename = "/w/loggerhead/opentag/R/M1.DSG"
calfilename = "/w/loggerhead/opentag/R/PRESSTMP.CAL"

datafile = file(filename, "rb")
calfile = file(calfilename, "rb")

# Calibration constants for pressure/temperature
# these are uints
PSENS = readBin(calfile, integer(), n=1, size=2, signed = FALSE, endian = "little");
POFF = readBin(calfile, integer(), n=1, size=2, signed = FALSE, endian = "little");
TCSENS = readBin(calfile, integer(), n=1, size=2, signed = FALSE, endian = "little");
TCOFF = readBin(calfile, integer(), n=1, size=2, signed = FALSE, endian = "little");
TREF = readBin(calfile, integer(), n=1, size=2, signed = FALSE, endian = "little");
TEMPSENS = readBin(calfile, integer(), n=1, size=2, signed = FALSE, endian = "little");
close(calfile)

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

INER_df <- data.frame("accelX" = numeric(0), "accelY" = numeric(0), "accelZ" = numeric(0), "magX" = numeric(0), "magY" = numeric(0), "magZ" = numeric(0), "gyroX" = numeric(0), "gyroY" = numeric(0), "gyroZ" = numeric(0))  # Inertial dataframe
PTMP_df8 <- data.frame()  # Pressure/Temperature dataframe with byte values
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
        dim(chunk) <- c(length(chunk) / numChan[cur_sid], numChan[cur_sid]) ## (rows, cols)
        INER_df = rbind(INER_df, data.frame(chunk))
      }
      if(SID[cur_sid] == "PTMP"){
        #dim(chunk) <- c(6,length(chunk) /  6, 6) ## (rows, cols)
        PTMP_df8 = rbind(PTMP_df8, data.frame(chunk))
      }
    }
}
close(datafile)

# calibrate values
srate=1000000.0/(period[1]);
accel_cal=16.0/4096.0;  #16 g/4096 (13 bit ADC)
gyro_cal=500.0/32768.0;  # 500 degrees per second (16-bit ADC)
mag_cal=1.0/1090.0;  #1090 LSB/Gauss

# Inertial headings calibration
names(INER_df) <- c("accelX", "accelY", "accelZ", "magX", "magY", "magZ", "gyroX", "gyroY", "gyroZ")
INER_df$accelX <- INER_df$accelX * accel_cal
INER_df$accelY <- INER_df$accelY * accel_cal
INER_df$accelZ <- INER_df$accelZ * accel_cal

INER_df$magX <- INER_df$magX * mag_cal
INER_df$magY <- INER_df$magY * mag_cal
INER_df$magZ <- INER_df$magZ * mag_cal

INER_df$gyroX <- INER_df$gyroX * gyro_cal
INER_df$gyroY <- INER_df$gyroY * gyro_cal
INER_df$gyroZ <- INER_df$gyroZ * gyro_cal

# Pressure/Temperature
# Combine values into 24-bit value and use calibration constants from files

n = nrow(PTMP_df8)
D1 = (PTMP_df8[seq(1, n, 6), 1] * 65536.0) + (PTMP_df8[seq(2, n, 6), 1] * 256.0) + PTMP_df8[seq(3, n, 6), 1]
D2 = (PTMP_df8[seq(4, n, 6), 1] * 65536.0) + (PTMP_df8[seq(5, n, 6), 1] * 256.0) + PTMP_df8[seq(6, n, 6), 1]
dT = D2 - TREF * 256.0
OFF = POFF * 65536.0 + (TCOFF * dT) / 128.0
SENS = PSENS * 32768.0 + (dT * TCSENS) / 256.0

 # mbar (i.e. a value of 1 = 1 mbar = ~1 cm depth resolution) 
PTMP_df = data.frame("temperature" = (2000.0 + dT * TEMPSENS / 8388608.0) / 100.0 , "pressure" = (D1 * SENS / 2097152.0 - OFF) / 81920.0)
#drops <- c("X1", "X2", "X3", "X4", "X5", "X6")
#PTMP_df = PTMP_df[ , !(names(PTMP_df) %in% drops)]