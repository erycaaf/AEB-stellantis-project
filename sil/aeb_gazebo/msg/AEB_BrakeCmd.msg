# AEB_BrakeCmd — CAN ID: 0x080 (128)
# Sender: AEB_Controller  |  Period: 10 ms  |  ASIL-B safety-critical
# Maps to DBC signals: BrakeRequest, BrakePressure, BrakeMode, AliveCounter, CRC

uint32  can_id           # 0x080
uint8   alive_counter    # rolling 0-15 (DBC: AliveCounter, bits 24-27)
uint8   crc              # 4-bit XOR of bytes 0-2 (DBC: CRC, bits 28-31)

bool    brake_request    # true = AEB brake active (DBC: BrakeRequest, bit 0)
float32 brake_pressure   # bar  (DBC: scale=0.1, range 0-3276.7, bits 1-15)

# BrakeMode: 0=Off 1=Warning 2=Partial_L1 3=Moderate_L2 4=Full_L3 5=PostBrakeHold
uint8   brake_mode       # (DBC: BrakeMode, bits 16-18)
