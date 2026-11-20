uint8 SQRT_MODE = 0
uint8 POLYNOMINAL_MODE = 1
uint8 RPM_MODE = 2

float32 voltage
float32 max_thrust

# POLYNOMINAL MODE: up to 4 Dimension
float32[5] polynominal

# RPM_MODE: thrust = krpm_to_thrust_ratio * krpm^krpm_to_thrust_exponent + krpm_to_thrust_bias [N]
float32 krpm_to_thrust_ratio
float32 krpm_to_thrust_exponent
float32 krpm_to_thrust_bias
