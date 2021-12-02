import numpy as np

cimport numpy as np
from libcpp cimport bool


cdef extern from "jaco2_rs485.h":
    cdef cppclass Jaco2:
        Jaco2(int display_error_level)
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
    cdef bool use_redis
    try:
        import redis
        r = redis.StrictRedis(host='localhost')
    except ImportError:
        pass

    def __cinit__(self, display_error_level, use_redis=False):
        self.use_redis = use_redis
        self.thisptr = new Jaco2(display_error_level)

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
        # TODO: doesn't work returning self.thisptr.torque_load
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
            if self.use_redis:
                self.r.set('q', '%.3f %.3f %.3f %.3f %.3f %.3f' %
                      tuple(self.thisptr.pos_rad))

    def SendTargetAnglesHand(self, bool open):
        self.thisptr.SendTargetAnglesHand(open)
