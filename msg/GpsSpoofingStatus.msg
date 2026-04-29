# BACH: CHANGES MADE HERE
uint64 timestamp              # time since system start (microseconds)

bool spoofing_detected

uint8 SPOOFING_ACTION_NONE = 0
uint8 SPOOFING_ACTION_REJECT_GPS = 1
uint8 SPOOFING_ACTION_FAILSAFE = 2
uint8 action

float32 confidence
