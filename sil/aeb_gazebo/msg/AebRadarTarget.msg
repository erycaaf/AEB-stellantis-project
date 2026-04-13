# AebRadarTarget — CAN ID: 0x120 (288)
# Sender: Radar_ECU  |  Period: 20 ms
# Maps to DBC signals: TargetDistance, RelativeSpeed, TTC, Confidence

uint32  can_id           # 0x120
uint8   alive_counter    # rolling 0-15

float32 target_distance  # m   (DBC: scale=0.01, range 0-655.35)
float32 relative_speed   # m/s (DBC: scale=0.01, offset=-327.68; negative = closing)
float32 ttc              # s   (DBC: scale=0.001, range 0-65.535)
uint8   confidence       # 0-15 (0=no target, 15=highest)
bool    sensor_fault     # FR-PER-007: true when 3 consecutive invalid cycles
