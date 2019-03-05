import matplotlib.pyplot as plt;
import numpy as np;
import scipy.optimize as opt;

def func1(x, k):
     return k*x**2 

def func2(x, a, b):
     return a*x + b 

rpm_set = np.array([600,720,840,960,1080,1200,1500,1800,2040,2220,2520,3060,4020,5000])
rpm_real = np.array([503, 692, 856, 917, 1038, 1157, 1443,1723, 1941, 2094, 2348, 2710, 3450, 4120])
m = np.array([12,18,28,30,42,56,94,147,179,218,273,352,600,860]) # [g]

# Change unit
# omega[rad/s] 
omega = rpm_real*2*np.pi/60
# force[N]
force = m/1000*9.81

fig = plt.figure("motor_thrust",figsize=(4.5,3.5))

optimizedParameters, pcov = opt.curve_fit(func1, omega, force)
k = optimizedParameters[0]

plt.plot(omega, force, "+", label="Data")
omega = np.linspace(np.min(omega),np.max(omega),50)
plt.plot(omega, func1(omega, k), label="$ F = %.5E \cdot \Omega^2$"%k)

plt.title("Thrust")
plt.xlabel("$\Omega$ [rad/s]")
plt.ylabel("$F$ [N]")
plt.legend(fancybox = True, loc="best")
plt.tight_layout()
fig.savefig("motor_thrust", dpi=300)

####################################################
fig = plt.figure("rpm",figsize=(4.5,3.5))

optimizedParameters, pcov = opt.curve_fit(func2, rpm_set, rpm_real)
a = optimizedParameters[0]
b = optimizedParameters[1]

plt.plot(rpm_set, rpm_real, "+", label="Data")
rpm_set = np.linspace(np.min(rpm_set),np.max(rpm_set),50)
plt.plot(rpm_set, func2(rpm_set, a, b), label="rpm_real = %.3f $\cdot$ rpm_set + %.3f"%(a,b))

plt.title("RPM")
plt.xlabel("RPM Set")
plt.ylabel("RPM Real")
plt.legend(fancybox = True, loc="best")
plt.tight_layout()
fig.savefig("rpm", dpi=300)

