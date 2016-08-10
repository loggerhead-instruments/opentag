# otDive

require(signal)
require(stats)
require(lubridate)
# files should be loaded with otload or from file


# Calculate depth
surfacePressure = 1013.25 # mbar
barPerMeter = 0.100693064 # bar at 1 m
PTMP$depth = (PTMP$pressure - surfacePressure) / (1000.0 * barPerMeter)

# Median filter depth
medFiltDur = 2 # median filter duration in seconds
medFiltPts = round(medFiltDur / periodS[2])
PTMP$depth = runmed(PTMP$depth, medFiltPts)

# Zero depth
PTMP$depth = min(PTMP$depth) - PTMP$depth

plot(PTMP$datetime, PTMP$depth, type='l')


# Calculate pitch, roll, yaw

# ODBA

# Dive detection

# Metrics by dive

