PI = 3.141592654
P_ANGLE_MIN = (-2.0 * PI)
P_ANGLE_MAX = (2.0 * PI)
V_MIN = -200.0
V_MAX = 200.0

KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 50.0
KI_MIN = 0.0
KI_MAX = 0.01

T_FF_MIN = -100.0
T_FF_MAX = 100.0
T_REF_MAX = 100.0
T_MIN = -100.0
T_MAX = 100.0

def float_to_uint(x,x_min,x_max,bits):
    span = x_max - x_min
    offset = x_min
    return (int) ((x-offset)*((float)((1<<bits)-1))/span)

