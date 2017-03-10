import redis
import timeit
import numpy as np
import pytry


starting =  (-0.17778635,  0.15433692,  0.19052379)
ending = (-0.07395785,  0.26072844,  0.69913439)
targets = ([starting, ending])
# starting = (0.4, -0.18, 0.85)
# targets = [(0.4, -0.18, 0.85), (-0.4, -0.18, 0.85)]

class TargetTest(pytry.Trial):
    def params(self):
        self.param('iterations', iterations=20)
        self.param('time per target', T=10)
        self.param('record period', dt=0.1)
        self.param('smooth transition', smooth=False)

    def evaluate(self, p):
        r = redis.StrictRedis(host='192.168.0.3')

        print('Going to start location')
        r.set('target_xyz', '%.3f, %.3f, %.3f' % starting)
        start = timeit.default_timer()
        now = start
        while now < start + p.T:
            now = timeit.default_timer()

        error_list = []
        total_error = []
        training_signal_list = []
        xyzs = []
        for iter in range(p.iterations):
            print('iteration %d' % iter)
            for i, target in enumerate(targets):
                print('going to target %s' % list(target))
                errors = []
                training_signals = []
                r.set('target_xyz', '%.3f, %.3f, %.3f' % target)
                start = timeit.default_timer()
                now = start
                while now < start + p.T:
                    if p.smooth:
                        tt = (np.cos((now - start)/p.T*np.pi)+1)/2
                        if i==0:
                            v = np.array(targets[0])*tt + np.array(targets[1])*(1-tt)
                        else:
                            v = np.array(targets[1])*tt + np.array(targets[0])*(1-tt)
                        r.set('target_xyz', '%.3f, %.3f, %.3f' % tuple(v))

                    if now > start + len(errors)*p.dt:

                        error = r.get('error')
                        if error is not None:
                            error = float(error.decode('ascii'))
                            errors.append(error)

                        training_signal = r.get('training_signal')
                        if training_signal is not None:
                            training_signal = training_signal.decode('ascii')
                            training_signal = [float(v) for v in training_signal[1:-1].split()]
                            training_signals.append(training_signal)

                        xyz = r.get('xyz')
                        if xyz is not None:
                            xyz = xyz.decode('ascii')
                            xyz = [float(v) for v in xyz[1:-1].split()]
                            xyzs.append(xyz)

                    now = timeit.default_timer()

                error_list.append(errors)
                total_error.append(sum(errors))
                training_signal_list.append(training_signals)

        print('Returning to start location')
        r.set('target_xyz', '%.3f, %.3f, %.3f' % starting)

        return dict(error_list=error_list,
                    training_signal_list=training_signal_list,
                    total_error=total_error,
                    xyzs=xyzs)
