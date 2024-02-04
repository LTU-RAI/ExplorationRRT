import opengen as og
import casadi.casadi as cs
import numpy as np

## Problem size

N = 15
dt = 1.0/20
nMAV = 1  #Number of MAVs to be included in the centralized scheme

## Weight matrices
# Qx = (8, 8, 40, 2, 2, 3, 8, 8)    
Qx = (4,4, 40, 4, 4, 3, 5, 5)    
P = 2*Qx; #final state weight
Ru = (3, 7, 7) # input weights
Rd = (3, 7, 7) # input rate weights

## Objective function generation
nu = 3; #Number of control inputs per MAV
ns = 8; #Number of states per MAV

np = nMAV*ns + nMAV*ns + nu + nu*nMAV  + 3 + 8
#print(np)
u = cs.SX.sym('u', nu*nMAV*N)
z0 = cs.SX.sym('z0', np)
#print(z0)
x = z0[0:nMAV*ns]
#print(x)
x_ref = z0[nMAV*ns:nMAV*ns + nMAV*ns]
#print(x_ref)
u_ref = z0[nMAV*ns + nMAV*ns:nMAV*ns + nMAV*ns + nu]
#print(u_ref)
u_old = z0[nMAV*ns + nMAV*ns + nu:nMAV*ns + nMAV*ns + nu + nMAV*nu]
f_nmhe = z0[nMAV*ns + nMAV*ns + nu + nu:nMAV*ns + nMAV*ns + nu + nu + 3]
Qx_adapt = z0[nMAV*ns + nMAV*ns + nu + nu + 3:nMAV*ns + nMAV*ns + nu + nu + 3 + 8]


cost = 0
c = 0

for i in range(0, N):
###State Cost 
    for j in range(0,nMAV):
        cost += Qx_adapt[0]*(x[ns*j]-x_ref[ns*j])**2 + Qx_adapt[1]*(x[ns*j+1]-x_ref[ns*j+1])**2 + Qx_adapt[2]*(x[ns*j+2]-x_ref[ns*j+2])**2 + Qx_adapt[3]*(x[ns*j+3]-x_ref[ns*j+3])**2 + Qx_adapt[4]*(x[ns*j+4]-x_ref[ns*j+4])**2 + Qx_adapt[5]*(x[ns*j+5]-x_ref[ns*j+5])**2 + Qx_adapt[6]*(x[ns*j+6]-x_ref[ns*j+6])**2 + Qx_adapt[7]*(x[ns*j+7]-x_ref[ns*j+7])**2

####Input Cost 
    u_n = u[(i*nMAV*nu):(i*nMAV*nu+nu*nMAV)]
    for j in range(0,nMAV):
        cost += Ru[0]*(u_n[nu*j] - u_ref[0])**2 + Ru[1]*(u_n[nu*j+1] - u_ref[1])**2 + Ru[2]*(u_n[nu*j+2] - u_ref[2])**2 #Input weights
        cost += Rd[0]*(u_n[nu*j] - u_old[nu*j])**2 + Rd[1]*(u_n[nu*j+1] - u_old[nu*j+1])**2 + Rd[2]*(u_n[nu*j+2] - u_old[nu*j+2])**2 #Input rate weights

        #Input rate constraints
        c = cs.vertcat(c, cs.fmax(0, u_n[nu*j+1] - u_old[nu*j+1] - 0.1))
        c = cs.vertcat(c, cs.fmax(0, u_old[nu*j+1] - u_n[nu*j+1] - 0.1))
        c = cs.vertcat(c, cs.fmax(0, u_n[nu*j+2] - u_old[nu*j+2] - 0.1))
        c = cs.vertcat(c, cs.fmax(0, u_old[nu*j+2] - u_n[nu*j+2] - 0.1))

    ####State update 
    u_old = u_n
    for j in range(0,nMAV):
        x[ns*j] = x[ns*j] + dt * x[ns*j+3]
        x[ns*j+1] = x[ns*j+1] + dt * x[ns*j+4]
        x[ns*j+2] = x[ns*j+2] + dt * x[ns*j+5]
        x[ns*j+3] = x[ns*j+3] + dt * (cs.sin(x[ns*j+7]) * cs.cos(x[ns*j+6]) * u_n[nu*j] - 0.1 * x[ns*j+3] + f_nmhe[0])
        x[ns*j+4] = x[ns*j+4] + dt * (-cs.sin(x[ns*j+6]) * u_n[nu*j] - 0.1*x[ns*j+4] + f_nmhe[1])
        x[ns*j+5] = x[ns*j+5] + dt * (cs.cos(x[ns*j+7]) * cs.cos(x[ns*j+6]) * u_n[nu*j] - 0.2 * x[ns*j+5] - 9.81 + f_nmhe[2])
        x[ns*j+6] = x[ns*j+6] + dt * ((1 / 0.20) * (u_n[nu*j+1] - x[ns*j+6]))
        x[ns*j+7] = x[ns*j+7] + dt * ((1 / 0.20) * (u_n[nu*j+2] - x[ns*j+7]))
        #print(x[ns*j])
    


umin = [3, -0.4, -0.4] * (N*nMAV)
umax = [15.5, 0.4, 0.4] * (N*nMAV)

bounds = og.constraints.Rectangle(umin, umax)
problem = og.builder.Problem(u, z0, cost).with_penalty_constraints(c) \
.with_constraints(bounds)

tcp_config = og.config.TcpServerConfiguration(bind_port=2769) 

build_config = og.config.BuildConfiguration()  \
.with_build_directory("MAV") \
.with_build_mode("release") \
.with_tcp_interface_config(tcp_config) 
#.with_build_c_bindings()
meta = og.config.OptimizerMeta()       \
.with_optimizer_name("errt_nmpc")
#.with_rebuild(True) 
solver_config = og.config.SolverConfiguration() \
.with_tolerance(1e-4) \
.with_initial_tolerance(1e-4) \
.with_max_duration_micros(40000) \
.with_max_outer_iterations(5) \
.with_penalty_weight_update_factor(2) \
.with_initial_penalty(1000.0) 
builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          build_config, solver_config) \
.with_verbosity_level(1)


builder.build()

# Use TCP server
# ------------------------------------
mng = og.tcp.OptimizerTcpManager('MAV/errt_nmpc')
mng.start()
x0 =   [2.0,2.0,1.0,0.0,0.0,0.0,0.0,0.0]
xref= [2.0,2.0,1.0,0.0,0.0,0.0,0.0,0.0]
uold =[9.81,0.0,0.0]
uref =[9.81,0.0,0.0]
qx_adapt = [5,5,20,4,4,4,5,5]

z0 = x0 + xref + uref + uold + [0,0,0] + qx_adapt
print(z0)
print(len(z0)) ##Length is equal to np
#obsdata = (0.0,0.0,1.0,1.0)
mng.ping()
solution = mng.call(z0, initial_guess=[9.81,0,0.0]*(N),buffer_len = 8*4096)
print(solution['solution'])
mng.kill()

