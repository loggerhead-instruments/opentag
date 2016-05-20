# opentag
Opentag is an arduino-compatible motion datalogger board

More information is at http://loggerhead.com

OpenTagGAMPT: Main control and sensor recording
cmd.pde: Reads recording settings from a script file
I2C.pde: Interface for I2C sensors

Sensors:
3D accelerometer (ADXL345)
3D magnetometer (HMC5883L)
3D gryoscope (IMU3000)
Pressure/Temperature (MS5803)
Real-time clock (DS1342)
Saltwater sense pin
Burn wire release: controls a FET switch to ground which can be used to corrode a stainless steel wire in seawater

Software supports:
- duty cycle recording
- wake on motion
- very low power running modes and sleep modes
- reset of firmware when unable to write to card