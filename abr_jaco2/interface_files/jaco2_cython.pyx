import numpy as np
cimport numpy as np
from libcpp cimport bool
import redis

r = redis.StrictRedis(host='127.0.0.1')

cdef extern from "jaco2_rs485.h":
    cdef cppclass Jaco2:
        Jaco2()
        # main functions
        void Connect()
        void InitForceMode()
        void InitPositionMode()
        void ApplyQSetup()
        int ApplyQ(float target_q[6])
        void ApplyQHand(bool open)
        void ApplyU(float u[6])
        void Disconnect()

        float pos[6]
        float posHand[3]
        float vel[6]
        float torque_load[6]
        float pos_rad[6]

cdef class pyJaco2:
    cdef Jaco2* thisptr # hold a C++ instance
    def __cinit__(self):
        self.thisptr = new Jaco2()

    def __dealloc__(self):
        del self.thisptr

    def Connect(self):
        self.thisptr.Connect()

    def InitForceMode(self):
        self.thisptr.InitForceMode()

    def InitPositionMode(self):
        self.thisptr.InitPositionMode()

    def ApplyQ(self, np.ndarray[float, mode="c"] target_q):
        TargetReached = 0
        self.thisptr.ApplyQSetup()
        while TargetReached < 6:
            TargetReached = self.thisptr.ApplyQ(&target_q[0])
            # print('pos_rad: ', self.thisptr.pos_rad)
            r.set('q', '%.3f %.3f %.3f %.3f %.3f %.3f' %
                  tuple(self.thisptr.pos_rad))

    def ApplyQHand(self, bool open):
        self.thisptr.ApplyQHand(open)

    def ApplyU(self, np.ndarray[float, mode="c"] u):
        self.thisptr.ApplyU(&u[0])

    def Disconnect(self):
        self.thisptr.Disconnect()

    def GetFeedback(self):
        feedback = {'q': self.thisptr.pos,
                    'dq': self.thisptr.vel}
        return feedback

    def GetTorqueLoad(self):
        # does not work when t_feedback = self.thisptr.torque_load TODO: INVESTIGATE
        t_feedback = {'torque_load' : self.thisptr.torque_load}
        return t_feedback
