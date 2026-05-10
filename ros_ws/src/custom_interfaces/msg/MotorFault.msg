std_msgs/Header header

string name

uint32 fault_code      # raw bitmask from motor firmware

# Decoded bits (faultSta register, section 3.3.7)
bool over_temp         # bit 0  — winding temp exceeded 135 °C
bool driver_ic         # bit 1  — driver chip fault
bool undervoltage      # bit 2  — bus voltage below 12 V
bool overvoltage       # bit 3  — bus voltage above 60 V
bool c_phase_oc        # bit 5  — C-phase overcurrent
bool encoder_uncal     # bit 7  — encoder not calibrated
bool hw_id_fault       # bit 8  — hardware ID mismatch
bool pos_init_fault    # bit 9  — position initialisation error
bool stall_overload    # bit 14 — stall I²t overload
bool a_phase_oc        # bit 16 — A-phase overcurrent
