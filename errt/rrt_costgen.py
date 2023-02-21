import opengen as og
import casadi.casadi as cs
#import matplotlib.pyplot as plt
import numpy as np
from trajectory import trajectory

## Problem size

N = 50
#dt = 1.0  / 2

## Weight matrices
Qx = (30,30,30, 2, 2,2, 4, 4)    
P = 2*Qx; #final state weight
Ru = (2,2,2); # input weights
Rd = (2, 2, 2); # input rate weights
QT = (100,100,100)

## Objective function generation
nu = 3
ns = 3
np = 8 + N*ns +  1
u = cs.SX.sym('u', nu*N)
z0 = cs.SX.sym('z0', np)
x = z0[0:8]
print(x)
n_xref = ns*N
x_ref_N = z0[8:(8+n_xref)]
print(x_ref_N)
dt = z0[(8+n_xref):(8+n_xref+1)]
cost = 0
c = 0
u_old = [9.81, 0 , 0]


for i in range(0, N):
####State Cost
    #x_ref = x_ref_N[(ns*i):(ns*i+ns)]
    #print(x_ref)
    
 #State weights

####Input Cost
    u_n = u[(3*i):3*i+3]

    cost += Ru[0]*(u_n[0] - 9.81)**2 + Ru[1]*(u_n[1])**2 + Ru[2]*(u_n[2])**2 #Input weights
    if i > 0:
        cost += Rd[0]*(u_n[0] - u_old[0])**2 + Rd[1]*(u_n[1] - u_old[1])**2 + Rd[2]*(u_n[2] - u_old[2])**2 #Input rate weights
    u_old = u_n


    #Penalty Constraint

    #c = cs.vertcat(c, cs.fmax(0, 0.36 - ((x[0] - obs_data[3*i])**2 + (x[1] - obs_data[3*i+1])**2 + (x[2] - obs_data[3*i+2])**2)))
    #c = cs.vertcat(c, cs.fmax(0, u_n[1] - u_old[1] - 0.05))
    #c = cs.vertcat(c, cs.fmax(0, u_old[1] - u_n[1]  - 0.05))
    #c = cs.vertcat(c, cs.fmax(0, u_n[2] - u_old[2] - 0.05))
    #c = cs.vertcat(c, cs.fmax(0, u_old[2] - u_n[2]- 0.05))

####State Update
    x[0] = x[0] + dt * x[3]
    x[1] = x[1] + dt * x[4]
    x[2] = x[2] + dt * x[5]
    x[3] = x[3] + dt * (cs.sin(x[7]) * cs.cos(x[6]) * u_n[0] - 1 * x[3])
    x[4] = x[4] + dt * (-cs.sin(x[6]) * u_n[0] - 1*x[4])
    x[5] = x[5] + dt * (cs.cos(x[7]) * cs.cos(x[6]) * u_n[0] - 1 * x[5] - 9.81)
    x[6] = x[6] + dt * ((1 / 0.5) * (u_n[1] - x[6]))
    x[7] = x[7] + dt * ((1 / 0.5) * (u_n[2] - x[7]))

    cost += Qx[0]*(x[0]-x_ref_N[3*i])**2 + Qx[1]*(x[1]-x_ref_N[3*i+1])**2 + Qx[2]*(x[2]-x_ref_N[3*i + 2])**2 + Qx[3]*(x[3])**2 + Qx[4]*(x[4])**2 + Qx[5]*(x[5])**2 + Qx[6]*(x[6])**2 + Qx[7]*(x[7])**2 

    #c = cs.vertcat(c, cs.fmax(0, 0.04 - ((x[0]-x_ref_N[3*i])**2 + (x[1]-x_ref_N[3*i+1])**2 + (x[2]-x_ref_N[3*i + 2])**2)))



cost += QT[0]*(x[0]-x_ref_N[3*(N-1)])**2 + QT[1]*(x[1]-x_ref_N[3*(N-1)+1])**2 + QT[2]*(x[2]-x_ref_N[3*(N-1) + 2])**2

umin = [5, -0.7, -0.7] * (nu*N)
#print(umin)
umax = [13.5, 0.7, 0.7] * (nu*N)
bounds = og.constraints.Rectangle(umin, umax)

tcp_config = og.config.TcpServerConfiguration(bind_port=3300)

problem = og.builder.Problem(u, z0, cost) \
.with_constraints(bounds)#.with_penalty_constraints(c)
build_config = og.config.BuildConfiguration()  \
.with_tcp_interface_config(tcp_config) \
.with_build_directory("MAV") \
.with_build_mode("release") \
.with_build_c_bindings() 
meta = og.config.OptimizerMeta()       \
.with_optimizer_name("rrt") 
#.with_rebuild(True) 
solver_config = og.config.SolverConfiguration() \
.with_tolerance(5e-5) \
.with_initial_tolerance(5e-5) \
.with_max_duration_micros(40000) \
.with_max_outer_iterations(4) \
.with_penalty_weight_update_factor(2) \
.with_initial_penalty(1000.0) 
builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          build_config, solver_config) \
.with_verbosity_level(1)
#builder.enable_tcp_interface()

builder.build()
# Use TCP server
# ------------------------------------
mng = og.tcp.OptimizerTcpManager('MAV/rrt')
mng.start()
dt = 1.0 / 2
x0 = [1.0,2.0,1.0,0,0,0,0,0]
xref = [1.0,8.0,1.0]*(N)
#obsdata = (0.0,0.0,1.0,0.0, 0.0, 0.0)
#obsdata = [0]*N*3
z0 = x0 + xref + [dt]
print(len(z0))
mng.ping()
solution = mng.call(z0, initial_guess=[9.81,0,0]*(N), buffer_len = 4*4096)
print(solution['solution'])
ustar = solution['solution']
[p_hist, bla, bleh] = trajectory(x0, ustar, 50, 0.5, xref)
print(p_hist)
mng.kill()
