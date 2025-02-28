################################
## Hardware Specific Settings ##
################################

# The type of mower you're using, not currently used but we might load a default config depending on this at some point
export OM_MOWER="HoverMower"

# Set to true to record your session
# export OM_ENABLE_RECORDING=True

# Offset between the point in between your wheels and the GPS antenna in m
#export OM_GPS_ANTENNA_OFFSET=0.3
# Angle which gets added to the compass heading, if your imu is in a rotated coordinate frame
#export OM_IMU_OFFSET=180.0

# IMU Calibration and filter settings
#export OM_MAG_BIAS_X=-0.027
#export OM_MAG_BIAS_Y=0.039
#export OM_MAG_BIAS_Z=0.025
#export OM_IMU_FILTER_GAIN=0.05

# Serial ports of your hardware devices
#export OM_LL_SERIAL_PORT=ttyAMA0
#export OM_LEFT_SERIAL_PORT=ttyAMA4
#export OM_RIGHT_SERIAL_PORT=ttyAMA2
#export OM_MOW_SERIAL_PORT=ttyAMA3
#export OM_GPS_SERIAL_PORT=ttyAMA1



################################
##        GPS Settings        ##
################################

# Relative Positioning vs LatLng coordinates
# If OM_USE_RELATIVE_POSITION=True, we're using the ublox NAVRELPOSNED messages as position.
# This makes your base station the map origin
# If OM_USE_RELATIVE_POSITION=False, we're using an arbitrary point as map origin. This point is called the DATUM point and
# needs to be set using OM_DATUM_LAT and OM_DATUM_LONG below.
# If you DON'T have your own base station (e.g. you're using an NTRIP service provider) you should use LAT_LONG positioning, else relative positioning
export OM_USE_RELATIVE_POSITION=True

# If needed, uncomment and set to coordinates near you (these default coordinates are somewhere in Germany).
# This will be your map origin!
# export OM_DATUM_LAT=48.13724720055111
# export OM_DATUM_LONG=11.575605219552623

# NTRIP Settings
# Set to False if using external radio.
#export OM_USE_NTRIP=True
#export OM_NTRIP_HOSTNAME=192.168.178.55
#export OM_NTRIP_PORT=2101
#export OM_NTRIP_USER=gps
#export OM_NTRIP_PASSWORD=gps
#export OM_NTRIP_ENDPOINT=BASE1

################################
##    Mower Logic Settings    ##
################################

# Voltages for battery to be considered full or empty
export OM_BATTERY_EMPTY_VOLTAGE=36.0
export OM_BATTERY_FULL_VOLTAGE=42.0

# Mower motor temperatures to stop and start mowing
export OM_MOWING_MOTOR_TEMP_HIGH=80.0
export OM_MOWING_MOTOR_TEMP_LOW=40.0

export OM_GPS_WAIT_TIME_SEC=10.0
export OM_GPS_TIMEOUT_SEC=5.0




# Mowing Behavior Settings
# True to enable mowing motor
export OM_ENABLE_MOWER=true

# True to start mowing automatically. If this is false, you need to start manually by pressing the start button
export OM_AUTOMATIC_START=false

export OM_OUTLINE_OFFSET=0.05
export OM_OUTLINE_COUNT=3
export OM_TOOL_WIDTH=0.15

export OM_DOCKING_DISTANCE=1.0
export OM_UNDOCK_DISTANCE=2.0
