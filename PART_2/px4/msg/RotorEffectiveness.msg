# Runtime effectiveness scaling of a single rotor, as applied by the control allocator.
# Set through CA_EFF_MOTOR / CA_EFF_SCALE. Published whenever the effectiveness matrix
# is rebuilt and once per second otherwise.

uint64 timestamp                # time since system start (microseconds)

uint8 motor                     # motor number (1-based, as in CA_EFF_MOTOR), 0 if none selected
float32 scale                   # scalar applied to that motor's effectiveness column [0, 1]
float32[6] effectiveness        # the motor's column as used by the allocator: roll, pitch, yaw torque, thrust x, y, z
