import numpy as np
cimport numpy as np

cdef extern from 'wrapped_code_9.h':
    void autofunc(double q0, double q1, double q2, double q3, double q4, double q5, double x, double y, double z, double *out_5395582705645376547)

def autofunc_c(double q0, double q1, double q2, double q3, double q4, double q5, double x, double y, double z):

    cdef np.ndarray[np.double_t, ndim=2] out_5395582705645376547 = np.empty((6,6))
    autofunc(q0, q1, q2, q3, q4, q5, x, y, z, <double*> out_5395582705645376547.data)
    return out_5395582705645376547