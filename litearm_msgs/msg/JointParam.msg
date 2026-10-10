# One axis of the firmware's joint parameter table.
#
# kp/kd/tau_max live in the same record as q_min/q_max, so one read returns both, even
# though the write services split them (set_joint_gains, set_joint_limits).

int32 index
float64 kp
float64 kd
float64 tau_max
float64 q_min
float64 q_max
