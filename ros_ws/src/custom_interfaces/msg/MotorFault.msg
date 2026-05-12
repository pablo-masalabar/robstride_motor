std_msgs/Header header

string name

uint32 fault_code      # raw fault bitmask (type-2 frame bits 21-16 / type-21 bytes 0-3)
uint32 warning_code    # raw warning bitmask (type-21 bytes 4-7)

# Decoded fault bits (section 4.1.8)
bool over_temp         # bit 0  — winding temp exceeded 135 °C
bool driver_ic         # bit 1  — driver chip fault
bool undervoltage      # bit 2  — bus voltage below 12 V
bool overvoltage       # bit 3  — bus voltage above 60 V
bool b_phase_oc        # bit 4  — B-phase overcurrent
bool c_phase_oc        # bit 5  — C-phase overcurrent
bool encoder_uncal     # bit 7  — encoder not calibrated
bool hw_id_fault       # bit 8  — hardware ID mismatch
bool pos_init_fault    # bit 9  — position initialisation error
bool stall_overload    # bit 14 — stall I²t overload
bool a_phase_oc        # bit 16 — A-phase overcurrent

# Decoded warning bits (section 4.1.8)
bool over_temp_warning # bit 0  — winding temp approaching 135 °C threshold
