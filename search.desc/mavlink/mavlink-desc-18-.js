searchState.loadedDescShard("mavlink", 18, "Returns the union of between the flags in <code>self</code> and <code>other</code>.\nReturns the union of between the flags in <code>self</code> and <code>other</code>.\nReturns the union of between the flags in <code>self</code> and <code>other</code>.\nReturns the union of between the flags in <code>self</code> and <code>other</code>.\nReturns the union of between the flags in <code>self</code> and <code>other</code>.\nReturns the union of between the flags in <code>self</code> and <code>other</code>.\nReturns the union of between the flags in <code>self</code> and <code>other</code>.\nReturns the union of between the flags in <code>self</code> and <code>other</code>.\nReturns the union of between the flags in <code>self</code> and <code>other</code>.\nReturns the union of between the flags in <code>self</code> and <code>other</code>.\nReturns the union of between the flags in <code>self</code> and <code>other</code>.\nReturns the union of between the flags in <code>self</code> and <code>other</code>.\nReturns the union of between the flags in <code>self</code> and <code>other</code>.\nTime until next update. Set to 0 if unknown or in data …\nTime since system boot..\nTime since the start-up of the node..\nTime since the start-up of the node..\nThe requested unique resource identifier (URI). It is not …\nMAVLink FTP URI for the general metadata file …\nVideo stream URI (TCP or RTSP URI ground station should …\nThe type of requested URI. 0 = a file via URL. 1 = a …\nTimestamp (UNIX time or time since system boot).\nTimestamp (UNIX time or time since system boot).\nTimestamp (UNIX time or time since system boot).\nTimestamp (UNIX time or since system boot).\nUsed capacity. If storage is not ready …\nUTC time in seconds since GPS epoch (Jan 6, 1980). If …\nVertical accuracy. NAN if unknown.\nAltitude uncertainty (standard deviation).\nAltitude uncertainty..\nAltitude uncertainty..\nEstimated delay of the speed data. 0 if unknown..\nNumber of valid points (up-to 5 waypoints are possible).\nNumber of valid control points (up-to 5 points are …\nFloating point value.\nDEBUG value.\nSigned integer value.\nMemory contents at specified address.\nVariability of wind in XY, 1-STD estimated from a 1 Hz …\nVariability of wind in Z, 1-STD estimated from a 1 Hz …\nGPS velocity in down direction in earth-fixed NED frame.\nGPS velocity in down direction in earth-fixed NED frame.\nTrue velocity in down direction in earth-fixed NED frame.\nGPS VDOP vertical dilution of position (unitless). If …\nGPS velocity in east direction in earth-fixed NED frame.\nGPS velocity in east direction in earth-fixed NED frame.\nTrue velocity in east direction in earth-fixed NED frame.\nGPS ground speed. If unknown, set to: UINT16_MAX.\ntarget velocity (0,0,0) for unknown.\nGPS ground speed. If unknown, set to: UINT16_MAX.\nGPS ground speed. If unknown, set to: UINT16_MAX.\nNorth-South velocity over ground in cm/s North +ve. If …\nGPS vertical speed in cm/s. If unknown set to INT16_MAX.\nVelocity accuracy. NAN if unknown.\nSpeed uncertainty (standard deviation).\nSpeed uncertainty..\nSpeed uncertainty..\nDown velocity of tracked object. NAN if unknown.\nEast velocity of tracked object. NAN if unknown.\nNorth velocity of tracked object. NAN if unknown.\nVelocity innovation test ratio.\nVariance of body velocity estimate.\nX-velocity of waypoint, set to NaN if not being used.\nY-velocity of waypoint, set to NaN if not being used.\nYaw rate, set to NaN if not being used.\nZ-velocity of waypoint, set to NaN if not being used.\nSpeed over ground.\nRow-major representation of a 6x6 velocity …\nID of the board vendor.\nName of the gimbal vendor..\nName of the camera vendor.\nVendor-specific status information..\nVersion code of the type variable. 0=unknown, type ignored …\nThe vertical velocity. Positive is up.\nCurrently active MAVLink version number * 100: v1.0 is …\n0: key as plaintext, 1-255: future, different …\nVertical speed 1-STD accuracy (0 if unknown).\nGPS vertical accuracy.\nThe accuracy of the vertical position..\nVertical Field of View (angle) where the distance …\nVertical field of view (NaN if unknown)..\nVibration levels on X-axis.\nVibration levels on Y-axis.\nVibration levels on Z-axis.\nCurrent status of video capturing (0: idle, 1: capture in …\nGPS velocity in north direction in earth-fixed NED frame.\nGPS velocity in north direction in earth-fixed NED frame.\nTrue velocity in north direction in earth-fixed NED frame.\nVoltage of the battery supplying the winch. NaN if unknown.\nVoltage measured from each ESC..\nBattery voltage, UINT16_MAX: Voltage not sent by autopilot.\nBattery voltage of cells 1 to 10 (see voltages_ext for …\nBattery voltages for cells 11 to 14. Cells above the valid …\nThe VTOL state if applicable. Is set to …\nGround X Speed (Latitude).\nX Speed.\nX Speed in NED (North, East, Down). NAN if unknown..\nX Speed.\nGround X Speed (Latitude, positive north).\nX velocity in NED frame.\nGround X speed (latitude, positive north).\nX velocity in NED frame.\nGround X Speed (Latitude).\nX linear speed.\nX velocity in NED frame.\nX velocity in NED frame.\nGround X Speed (Latitude).\nGround Y Speed (Longitude).\nY Speed.\nY Speed in NED (North, East, Down). NAN if unknown..\nY Speed.\nGround Y Speed (Longitude, positive east).\nY velocity in NED frame.\nGround Y speed (longitude, positive east).\nY velocity in NED frame.\nGround Y Speed (Longitude).\nY linear speed.\nY velocity in NED frame.\nY velocity in NED frame.\nGround Y Speed (Longitude).\nGround Z Speed (Altitude).\nZ Speed.\nZ Speed in NED (North, East, Down). NAN if unknown..\nZ Speed.\nGround Z Speed (Altitude, positive down).\nZ velocity in NED frame.\nGround Z speed (altitude, positive down).\nZ velocity in NED frame.\nGround Z Speed (Altitude).\nZ linear speed.\nZ velocity in NED frame.\nZ velocity in NED frame.\nGround Z Speed (Altitude).\nBattery weight. 0: field not provided..\nWidth of a matrix or image..\nAltitude (MSL) that this measurement was taken at (NAN if …\nWind heading.\nWind in North (NED) direction (NAN if unknown).\nWind in East (NED) direction (NAN if unknown).\nWind in down (NED) direction (NAN if unknown).\nWindspeed.\nGPS Week Number of last baseline.\nGPS Week Number of last baseline.\nDistance to active waypoint.\ndistance to target.\ncurrent waypoint number.\nCurrent waypoint number.\nWrite speed..\nX Position of the landing target in MAV_FRAME.\nLocal X position of this position in the local coordinate …\nX Position.\nx.\nX-axis, normalized to the range [-1000,1000]. A value of …\nPARAM5 / local: X coordinate, global: latitude.\nPARAM5 / local: x position in meters * 1e4, global: …\nPARAM5 / local: x position in meters * 1e4, global: …\nX position (NED).\nX coordinate of center point. Coordinate system depends on …\nX Position.\nGlobal X position.\nX Position.\nX Position in NED frame.\nLocal X position.\nLocal X position of this position in the local coordinate …\nX Position.\nX Position in NED frame.\nGlobal X speed.\nGlobal X position.\nX acceleration in body frame.\nX position in local frame.\nX velocity in body frame.\nX acceleration.\nX acceleration.\nX acceleration.\nX acceleration.\nX acceleration (raw).\nX acceleration.\nX acceleration.\nX acceleration.\nX acceleration.\nAngular speed around X axis.\nAngular speed around X axis.\nAngular speed around X axis in body frame.\nAngular speed around X axis (raw).\nAngular speed around X axis.\nAngular speed around X axis.\nAngular speed around X axis.\nX Magnetic field.\nX Magnetic field.\nX Magnetic field.\nX Magnetic field (raw).\nX Magnetic field.\nX Magnetic field.\nCurrent crosstrack error on x-y plane.\nY Position of the landing target in MAV_FRAME.\nLocal Y position of this position in the local coordinate …\nY Position.\ny.\nY-axis, normalized to the range [-1000,1000]. A value of …\nPARAM6 / local: Y coordinate, global: longitude.\nPARAM6 / local: y position in meters * 1e4, global: …\nPARAM6 / y position: local: x position in meters * 1e4, …\nY position (NED).\nY coordinate of center point.  Coordinate system depends …\nY Position.\nGlobal Y position.\nY Position.\nY Position in NED frame.\nLocal Y position.\nLocal Y position of this position in the local coordinate …\nY Position.\nY Position in NED frame.\nGlobal Y speed.\nGlobal Y position.\nY acceleration in body frame.\nY position in local frame.\nY velocity in body frame.\nY acceleration.\nY acceleration.\nY acceleration.\nY acceleration.\nY acceleration (raw).\nY acceleration.\nY acceleration.\nY acceleration.\nY acceleration.\nYaw relative to vehicle (set to NaN for invalid)..\nYaw angle unitless (-1..1, positive: to the right, …\nYaw angle.\nYaw.\nyaw setpoint.\nYaw angle.\nYaw of vehicle relative to Earth’s North, zero means not …\nyaw setpoint.\nYaw angle (-pi..+pi).\nYaw angle.\nYaw in earth frame from north. Use 0 if this GPS does not …\nYaw of vehicle relative to Earth’s North, zero means not …\nYaw in earth frame from north. Use 0 if this GPS does not …\nYaw angle (positive: to the right, negative: to the left, …\nyaw setpoint.\nyaw setpoint.\nYaw angle.\nAttitude yaw expressed as Euler angles, not recommended …\nDesired yaw rate.\nYaw in absolute frame relative to Earth’s North, north …\nMaximum hardware yaw angle (positive: to the right, …\nMaximum yaw angle (positive: to the right, negative: to …\nMinimum hardware yaw angle (positive: to the right, …\nMinimum yaw angle (positive: to the right, negative: to …\nYaw angular rate unitless (-1..1, positive: to the right, …\nyaw rate setpoint.\nyaw rate setpoint.\nAngular rate in yaw axis.\nYaw angular rate (positive: to the right, negative: to the …\nyaw rate setpoint.\nyaw rate setpoint.\nControl output -1 .. 1.\nBody frame yaw / psi angular speed.\nYaw angular speed.\nBody frame yaw / psi angular speed.\nYaw angular speed.\nYaw angular speed.\nYaw angular speed.\nAngular speed around Y axis.\nAngular speed around Y axis.\nAngular speed around Y axis in body frame.\nAngular speed around Y axis (raw).\nAngular speed around Y axis.\nAngular speed around Y axis.\nAngular speed around Y axis.\nY Magnetic field.\nY Magnetic field.\nY Magnetic field.\nY Magnetic field (raw).\nY Magnetic field.\nY Magnetic field.\nZ Position of the landing target in MAV_FRAME.\nLocal Z position of this position in the local coordinate …\nZ Position.\nz.\nZ-axis, normalized to the range [-1000,1000]. A value of …\nPARAM7 / local: Z coordinate, global: altitude (relative …\nPARAM7 / z position: global: altitude in meters (relative …\nPARAM7 / z position: global: altitude in meters (relative …\nZ position (NED).\nAltitude of center point. Coordinate system depends on …\nZ Position.\nGlobal Z position.\nZ Position.\nZ Position in NED frame (note, altitude is negative in …\nLocal Z position.\nLocal Z position of this position in the local coordinate …\nZ Position.\nZ Position in NED frame (note, altitude is negative in …\nGlobal Z speed.\nGlobal Z position.\nZ acceleration in body frame.\nZ position in local frame.\nZ velocity in body frame.\nZ acceleration.\nZ acceleration.\nZ acceleration.\nZ acceleration.\nZ acceleration (raw).\nZ acceleration.\nZ acceleration.\nZ acceleration.\nZ acceleration.\nAngular speed around Z axis.\nAngular speed around Z axis.\nAngular speed around Z axis in body frame.\nAngular speed around Z axis (raw).\nAngular speed around Z axis.\nAngular speed around Z axis.\nAngular speed around Z axis.\nZ Magnetic field.\nZ Magnetic field.\nZ Magnetic field.\nZ Magnetic field (raw).\nZ Magnetic field.\nZ Magnetic field.\nCurrent zoom level as a percentage of the full range (0.0 …\nA trait very similar to <code>Default</code> but is only implemented …\nRemoves the trailing zeroes in the payload")