# AebAlert — CAN ID: 0x300 (768)
# Sender: AEB_Controller  |  Event-driven (published on state change only)
# Maps to DBC signals: AlertType, AlertActive, BuzzerCmd
#
# alert_type values: 0=NONE 1=VISUAL 2=AUDIBLE 3=BOTH
# buzzer_cmd values: 0=Off 1=SingleBeep 2=DoubleBeep 3=ContinuousBeep 4=FastPulse 5=Chirp

uint32  can_id           # 0x300

uint8   alert_type       # 0-3 (DBC: AlertType, bits 0-7)
bool    alert_active     # (DBC: AlertActive, bit 8)
bool    visual_active    # convenience: true when alert_type in {1,3}
bool    audible_active   # convenience: true when alert_type in {2,3}
uint8   buzzer_cmd       # 0-5 (DBC: BuzzerCmd, bits 9-11)
