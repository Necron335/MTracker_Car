# Internal ROS adapter contract, not a serial wire layout.
# CAR trace IDs: 15 (reference), 1 (velocity), 13 (position error), 5 (output).
# All fields must come from one firmware capture, with verified physical signs.
uint32 index
float64 time_s
float64 reference_velocity_m_s
float64 velocity_m_s
float64 left_position_error_rad
float64 left_output
