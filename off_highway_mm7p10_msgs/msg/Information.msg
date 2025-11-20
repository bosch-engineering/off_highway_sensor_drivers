std_msgs/Header header

# 4-bit status: bit0=IMU not available, bit1=signal failure, bit2=initialization, bit3=reserved
uint8 yaw_rate_stat
uint8 roll_rate_stat
uint8 pitch_rate_stat
uint8 ax_stat
uint8 ay_stat
uint8 az_stat

# Temperature in raw format: 0x0=-50°C, 0xC8=150°C, 0xFF=invalid/CRC error, 0xC9>150°C
uint8 temp_rate_z

# Hardware index from sensor
uint8 hw_index
