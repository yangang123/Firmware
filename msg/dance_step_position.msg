 uint64 timestamp # time since system start (microseconds)
 uint32 time_boot_ms #< [ms] Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.*/
 int32 lat_int #< [degE7] X Position in WGS84 frame*/
 int32 lon_int #< [degE7] Y Position in WGS84 frame*/
 float32 alt #< [m] Altitude (MSL, Relative to home, or AGL - depending on frame)*/
 float32 vx #< [m/s] X velocity in NED frame*/
 float32 vy #< [m/s] Y velocity in NED frame*/
 float32 vz #< [m/s] Z velocity in NED frame*/
 float32 afx #< [m/s/s] X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
 float32 afy #< [m/s/s] Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
 float32 afz #< [m/s/s] Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
 float32 yaw #< [rad] yaw setpoint*/
 float32 yaw_rate #< [rad/s] yaw rate setpoint*/
 uint16 type_mask #<  Bitmap to indicate which dimensions should be ignored by the vehicle.*/
 uint8 target_system #<  System ID*/
 uint8 target_component #<  Component ID*/
 uint8 coordinate_frame #<  Valid options are
