# One snapshot of the arm as litearm_driver sees it.
#
# Everything here is either a local SDK snapshot or the last cached status frame. The
# publisher never sends a frame to fill this message, so subscribing cannot disturb a
# command that is already in flight. The two host-side views (speed_scaling, zero_g_*)
# say so explicitly; they are not read back from the firmware.

std_msgs/Header header

# Link and lifecycle.
bool connected              # reader thread alive and the last status frame is fresh
bool in_dfu                 # terminal DFU state; the CDC port is gone until a new Arm object
bool enabled                # firmware reports the motors enabled
bool faulted                # firmware FAULT flag, EMERGENCY, or any axis cut off
bool cart_busy              # firmware flags bit 10: a cartesian plan is planning or playing
uint16 joint_fault          # per-axis fault bitmap, bit i = axis i (firmware 1.5.0 and newer)
bool drop_hold_inferred     # inference, not a reported bit: joint_fault != 0 implies the
                            # firmware's rigid-hold latch may be engaged. false is certain;
                            # true can also mean a single-axis-only fault. reset() or
                            # clear_faults() are the only ways to clear it.

# Firmware identity and mode.
string firmware         # e.g. "Litearm1.8.0-7J"
uint16 mode
string mode_name
uint16 flags
string last_reset_reason  # "normal", "iwdg-rst", or empty when no boot signature arrived

# Motion configuration, host-side view.
uint8 speed_scaling     # percent last written by set_speed_scaling (0..100)
bool zero_g_active      # the SDK's zero-gravity keep-alive thread is running
string zero_g_error     # non-empty when that keep-alive thread died

# Per-axis diagnostics, all indexed in joint_names order.
string[] joint_names
float64[] temperature_mos
float64[] temperature_coil
uint8[] joint_error_code
float64 feedback_age    # seconds since the last status frame; negative = none yet

# Licence record, cached from the last read. This publisher never refreshes it.
bool license_valid
uint8 license_state
string license_state_name
string license_uid
