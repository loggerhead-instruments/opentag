
filename = "/w/loggerhead/opentag/R/M1.DSG"

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

INER_df = data.frame()  # Inertial dataframe
PTMP_df = data.frame()  # Pressure/Temperature dataframe
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
        chunk = readBin(datafile, integer(), n = nsamples * 3, size = 1, endian = "little")
      }
      if(dForm[cur_sid] == 4){
        nsamples = nBytes[cur_sid] / 4
        chunk = readBin(datafile, numeric(), n = nsamples, size = 4, endian = "little")
      }
      # add to appropriate dataframe as read in
      dim(chunk) <- c(length(chunk) / numChan[cur_sid], numChan[cur_sid]) ## (rows, cols)
      if(SID[cur_sid] == "INER"){
        INER_df = rbind(INER_df, data.frame(chunk))
      }
      if(SID[cur_sid] == "PTMP"){
        PTMP_df = rbind(PTMP_df, data.frame(chunk))
      }
    }
}
close(datafile)

# calibrate values

