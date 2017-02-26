import numpy as np
cimport numpy as np

cdef extern from 'wrapped_code_3.h':
    void autofunc(double q0, double q1, double q2, double q3, double q4, double q5, double *out_2186966456938278944)

def autofunc_c(double q0, double q1, double q2, double q3, double q4, double q5):

    cdef np.ndarray[np.double_t, ndim=2] out_2186966456938278944 = np.empty((6,6))
    autofunc(q0, q1, q2, q3, q4, q5, <double*> out_2186966456938278944.data)
    return out_2186966456938278944