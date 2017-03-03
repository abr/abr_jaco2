import numpy as np
cimport numpy as np

cdef extern from 'wrapped_code_4.h':
    void autofunc(double q0, double q1, double q2, double q3, double q4, double q5, double *out_3049059153008242927)

def autofunc_c(double q0, double q1, double q2, double q3, double q4, double q5):

    cdef np.ndarray[np.double_t, ndim=2] out_3049059153008242927 = np.empty((6,1))
    autofunc(q0, q1, q2, q3, q4, q5, <double*> out_3049059153008242927.data)
    return out_3049059153008242927