import numpy as np
cimport numpy as np
from libcpp cimport bool
import redis

# TODO: clean this up jeeeez
r = redis.StrictRedis(host='127.0.0.1')

cdef extern from "jaco2_rs485.h":
    cdef cppclass Jaco2:
        Jaco2()
        # main functions
        void Connect()
        void Disconnect()
        void InitForceMode()
        void InitPositionMode()
        int SendTargetAngles(float q_target[6])
        void SendTargetAnglesHand(bool open)
        void SendTargetAnglesSetup()
        void SendForces(float u[6])

        float pos[6]
        float posHand[3]
        float pos_rad[6]
        float torque_load[6]
        float vel[6]

cdef class pyJaco2:
    cdef Jaco2* thisptr # hold a C++ instance
    def __cinit__(self):
        self.thisptr = new Jaco2()

    def __dealloc__(self):
        del self.thisptr

    def Connect(self):
        self.thisptr.Connect()

    def Disconnect(self):
        self.thisptr.Disconnect()

    def GetFeedback(self):
        return {'q': self.thisptr.pos,
                'dq': self.thisptr.vel}

    def GetTorqueLoad(self):
        # TODO: why doesn't it work to return self.thisptr.torque_load
        return {'torque_load' : self.thisptr.torque_load}

    def InitForceMode(self):
        self.thisptr.InitForceMode()

    def InitPositionMode(self):
        self.thisptr.InitPositionMode()

    def SendForces(self, np.ndarray[float, mode="c"] u):
        self.thisptr.SendForces(&u[0])

    def SendTargetAngles(self, np.ndarray[float, mode="c"] q_target):
        target_reached = 0
        self.thisptr.SendTargetAnglesSetup()
        while target_reached < 6:
            target_reached = self.thisptr.SendTargetAngles(&q_target[0])
            # TODO: make this optional
            r.set('q', '%.3f %.3f %.3f %.3f %.3f %.3f' %
                  tuple(self.thisptr.pos_rad))

    def SendTargetAnglesHand(self, bool open):
        self.thisptr.SendTargetAnglesHand(open)
