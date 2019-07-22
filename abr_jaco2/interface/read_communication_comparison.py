import numpy as np
test1 = "loop_times_0x200.txt"
test2 = "loop_times_0x0014.txt"
f1 = open(test1, "r")
data1 = f1.readlines()
times1 = []
print(test1)
for ii, time in enumerate(data1):
    if float(time) < 0:
        print(ii)
        print(time)
    else:
        times1.append(float(time))

f2 = open(test2, "r")
data2 = f2.readlines()
times2 = []
print(test2)
for ii, time in enumerate(data2):
    if float(time) < 0:
        print(ii)
        print(time)
    else:
        times2.append(float(time))
print(test1)
print('MEAN: ', np.mean(times1))
print('VARIANCE: ', np.var(times1))
print(test2)
print('MEAN: ', np.mean(times2))
print('VARIANCE: ', np.var(times2))
