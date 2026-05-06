# AebEgoVehicle — CAN ID: 0x100 (256)
# Sender: Vehicle_ECU  |  Period: 10 ms
# Maps to DBC signals: VehicleSpeed, LongAccel, YawRate, SteeringAngle

uint32  can_id           # 0x100
uint8   alive_counter    # rolling 0-15

float32 vehicle_speed    # m/s  (DBC: scale=0.01, range 0-655.35)
float32 long_accel       # m/s² (DBC: scale=0.001, offset=-32; negative = braking)
float32 yaw_rate         # deg/s
float32 steering_angle   # deg  (positive = left)
bool    brake_pedal      # driver brake pedal pressed (driver override)
