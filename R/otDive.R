# otDive

require(signal)
require(stats)
require(lubridate)
# files should be loaded with otload or from file


# Calculate depth
surfacePressure = 1013.25 # mbar
barPerMeter = 0.100693064 # bar at 1 m
PTMP$depth = (PTMP$pressure - surfacePressure) / (1000.0 * barPerMeter)

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

# ODBA


# Dive detection


# Metrics by dive

