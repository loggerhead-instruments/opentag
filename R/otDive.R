# otDive
# OpenTag files should be loaded with otload or from file

require(signal)
require(stats)
require(lubridate)

# Calculate depth
surfacePressure = 1013.25 # mbar
barPerMeter = 0.100693064 # bar at 1 m
PTMP$depth = (PTMP$pressure - surfacePressure) / (1000.0 * barPerMeter)

plot(PTMP$datetime, PTMP$depth, type='l')

# Median filter depth to remove bad values
medFiltDur = 2 # median filter duration in seconds
medFiltPts = round(medFiltDur / periodS[2])
PTMP$depth = runmed(PTMP$depth, medFiltPts)

# Zero depth -- this assumes that minimum pressure of time series is at surface
PTMP$depth = min(PTMP$depth) - PTMP$depth

plot(PTMP$datetime, PTMP$depth, type='l')

# Calculate pitch, roll, yaw
radPerDeg = 0.0174532925
# roll
phi = atan2(INER$accelY, INER$accelZ)
sinAngle = sin(phi)
cosAngle = cos(phi)

# de-rotate by roll angle
Bfy = (INER$magY * cosAngle) - (INER$magZ * sinAngle)
Bz = (INER$magY * sinAngle) + (INER$magZ * cosAngle)
Gz = INER$accelY * sinAngle + INER$accelZ * cosAngle

# theta = pitch angle (-90 to 90 degrees)
theta = atan(-INER$accelX / Gz)
sinAngle = sin(theta)
cosAngle = cos(theta)

# de-rotate by pitch angle theta
Bfx = (INER$magX * cosAngle) + (Bz * sinAngle)
Bfz = (-INER$magX * sinAngle) + (Bz * cosAngle)

# Psi = yaw = heading
psi = atan2(-Bfy, Bfx)

INER$pitch = theta / radPerDeg
INER$Roll = phi / radPerDeg
INER$Yaw = 180 + (psi / radPerDeg)

# VDBA
smoothDur = 2 # remove moving average of smoothDur seconds
n = round(smoothDur / periodS[1])
#moving average centered around lag 0
INER$VDBA = sqrt((INER$accelX - stats::filter(INER$accelX, rep(1/n, n), sides = 2))^2 +
                (INER$accelY - stats::filter(INER$accelY, rep(1/n, n), sides = 2))^2 +
                (INER$accelZ - stats::filter(INER$accelZ, rep(1/n, n), sides = 2))^2)

# Dive detection
diveDepth = -1
dives = PTMP$depth < diveDepth
diffDives = diff(dives)

# # number dives and surface intervals incrementally
# diveNum = 1
# surfaceNum = 1
# 
# index = 0
# lastDiveTime = dive.index[0]
# firstDive = 1
# diveLabel = ''
# curDiveTime = ''
# timediff = dive.index[index] - dive.index[index]
# 
# # vectorized version
# 
# surface = dive['DepthDiff'][(dive['DepthDiff']==-1)] 
# divedive = dive['DepthDiff'][(dive['DepthDiff']==1)]
# 
# dive['DiveStartTime'] = pd.DataFrame({'DiveTime': dive.index[(dive['DepthDiff']==1) | (dive['DepthDiff']==-1)]}, index=dive.index[(dive['DepthDiff']==1) | (dive['DepthDiff']==-1)])
# # increment values
# ss = surface * (np.arange(surface.size)+1)
# dd = divedive  * (np.arange(divedive.size)+1)
# 
# dive['dive'] = pd.concat([ss,dd])
# 
# dive.fillna(method='ffill', inplace=True)

# Give each dive a dive number


# Metrics by dive 
# look at aggregate function

