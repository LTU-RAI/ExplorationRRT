def trajectory(x, u, N, dt,trajectory):
    #Based on the initial condition and optimized trajectory u, computed the path as (x,y,z).
    #Calculate the dynamic costs based on selected weights  
    import math
    import numpy as np
    ns = 8
    p_traj = []
    v_traj = []
    cost = 0
    ## Weight matrices
    Qx = (5,5,5, 0, 0,0, 5, 5)    
    #P = 2*Qx; #final state weight
    Ru = (100,100,100); # input weights
    Rd = (100, 100, 100); # input rate weights
    #print(x, u, N, dt)
    u_old = [9.81,0,0]
    for i in range(0,N):
####State costs
        position = trajectory[(3*i):(3*i+3)]
        #print(x_ref)
    
 #State weights
####Input Cost
        u_n = u[(3*i):3*i+3]
        cost += Ru[0]*(u_n[0] - 9.81)**2 + Ru[1]*(u_n[1])**2 + Ru[2]*(u_n[2])**2 #Input weights
        #print(Ru[0], u_n[0], 9.81, Ru[1], u_n[1], 0, Ru[2], u_n[2], 0)
        cost += Rd[0]*(u_n[0] - u_old[0])**2 + Rd[1]*(u_n[1] - u_old[1])**2 + Rd[2]*(u_n[2] - u_old[2])**2 #Input rate weights
        #print(Rd[0], u_n[0], u_old[0], Rd[1], u_n[1], u_old[1], Rd[2], u_n[2], u_old[2]);
        u_old = u_n
        x[0] = x[0] + dt * x[3]
        x[1] = x[1] + dt * x[4] 
        x[2] = x[2] + dt * x[5] 
        x[3] = x[3] + dt * (math.sin(x[7]) * math.cos(x[6]) * u[3*i] - 1 * x[3]) 
        x[4] = x[4] + dt * (-math.sin(x[6]) * u[3*i] - 1*x[4])
        x[5] = x[5] + dt * (math.cos(x[7]) * math.cos(x[6]) * u[3*i] - 1 * x[5] - 9.81) 
        x[6] = x[6] + dt * ((1 / 0.5) * (u[3*i+1] - x[6])) 
        x[7] = x[7] + dt * ((1 / 0.5) * (u[3*i+2] - x[7]))
        cost += Qx[0]*(x[0]-position[0])**2 + Qx[1]*(x[1]-position[1])**2 + Qx[2]*(x[2]-position[2])**2 + Qx[3]*(x[3])**2 + Qx[4]*(x[4])**2 + Qx[5]*(x[5])**2 + Qx[6]*(x[6])**2 + Qx[7]*(x[7])**2
        print("This is good");
        print(Qx[0], x[0], position[0], Qx[1], x[1], position[1], Qx[2], x[2], position[2], Qx[3], x[3], Qx[4], x[4], Qx[5], x[5], Qx[6], x[6], Qx[7], x[7])
        p_traj = p_traj + [[x[0],x[1],x[2]]]
        v_traj = v_traj + [[x[3],x[4],x[5]]]
    #print(cost)
    #print(p_traj)
    return(p_traj, v_traj, cost)
