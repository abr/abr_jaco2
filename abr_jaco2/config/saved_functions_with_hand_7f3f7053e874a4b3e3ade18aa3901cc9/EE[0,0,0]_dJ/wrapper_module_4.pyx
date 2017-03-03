import numpy as np
cimport numpy as np

cdef extern from 'wrapped_code_4.h':
    void autofunc(double q0, double q1, double q2, double q3, double q4, double q5, double dq0, double dq1, double dq2, double dq3, double dq4, double dq5, double x, double y, double z, double *out_7522350958389737753)

def autofunc_c(double q0, double q1, double q2, double q3, double q4, double q5, double dq0, double dq1, double dq2, double dq3, double dq4, double dq5, double x, double y, double z):

    cdef np.ndarray[np.double_t, ndim=2] out_7522350958389737753 = np.empty((6,6))
    autofunc(q0, q1, q2, q3, q4, q5, dq0, dq1, dq2, dq3, dq4, dq5, x, y, z, <double*> out_7522350958389737753.data)
    return out_7522350958389737753