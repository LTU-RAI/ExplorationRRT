import opengen as og
import casadi.casadi as cs
#import matplotlib.pyplot as plt
import numpy as np
import math 
from trajectory import trajectory
mng = og.tcp.OptimizerTcpManager('MAV/rrt')
mng.start()
N = 50
dt = 1.0 / 2
x0 = [0,0,0,0,0,0,0,0]
xref = [0,0,0]*N

for i in range(0,N):
    xref[3*i] = i*0.5
    xref[3*i+1] = i*0.5
    xref[3*i+2] = 0

#obsdata = (0.0,0.0,1.0,0.0, 0.0, 0.0)
#obsdata = [0]*N*3
z0 = x0 + xref + [dt]
print(z0);
mng.ping()
solution = mng.call(z0, initial_guess=[9.81,0,0]*(N), buffer_len = 4*4096)
#print(solution['solution'])
solution_data = solution.get();
#print(error_msg);
ustar = solution_data.solution
print(ustar)
[p_traj, v_traj, cost] = trajectory(x0, ustar, 50, 0.5, xref)
#print(len(p_traj))
print(cost)

px = [0]*N
py = [0]*N
for i in range(0,N):
    px[i] = p_traj[i][0]
    py[i] = p_traj[i][1]

px_ref = [0]*N	
py_ref = [0]*N
for i in range(0,N):
     px_ref[i] = xref[3*i]
     py_ref[i] = xref[3*i+1]

import matplotlib.pyplot as plt
plt.plot(px,py,px_ref,py_ref)
plt.show()

mng.kill()
