import numpy as np
cimport numpy as np

cdef extern from 'wrapped_code_0.h':
    void autofunc(double q0, double q1, double q2, double q3, double q4, double q5, double *out_5801400662906225431)

def autofunc_c(double q0, double q1, double q2, double q3, double q4, double q5):

    cdef np.ndarray[np.double_t, ndim=2] out_5801400662906225431 = np.empty((3,3))
    autofunc(q0, q1, q2, q3, q4, q5, <double*> out_5801400662906225431.data)
    return out_5801400662906225431