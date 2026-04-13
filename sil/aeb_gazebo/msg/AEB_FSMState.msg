# AEB_FSMState — CAN ID: 0x200 (512)
# Sender: AEB_Controller  |  Period: 50 ms
# Maps to DBC signals: FSMState, AlertLevel, BrakeActive, TTCThreshold
#
# FSMState values: 0=OFF 1=STANDBY 2=WARNING 3=BRAKE_L1 4=BRAKE_L2 5=BRAKE_L3 6=POST_BRAKE
# AlertLevel values: 0=NONE 1=WARNING 2=BRAKE_LOW 3=BRAKE_HIGH

uint32  can_id           # 0x200

uint8   fsm_state        # enum 0-6 (DBC: FSMState, bits 0-7)
string  fsm_state_name   # human-readable name derived from fsm_state
uint8   alert_level      # 0-3    (DBC: AlertLevel, bits 8-15)
bool    brake_active     # (DBC: BrakeActive, bits 16-23)
float32 ttc_threshold    # s (DBC: scale=0.1, range 0-25.5, bits 24-31)
