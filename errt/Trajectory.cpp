std::tuple<std::list<double>, double, std::list<double>> trajectory(std::list<double> x, double* u, double N, double dt, std::list<double> nmpc_ref){
    // Based on the initial condition and optimized trajectory u, computed the path as (x,y,z).
    // Calculate the dynamic costs based on selected weights  
    double ns = 8;
    std::list<double> p_traj{};
    std::list<double> v_traj{};
    double cost = 0;
    // Weight matrices
    std::list<double> Qx = {5,5,5, 0, 0,0, 5, 5};
    // P = 2*Qx; #final state weight
    std::list<double> Ru = {15,15,15}; // input weights
    std::list<double> Rd = {10, 10, 10}; // input rate weights
    // print(x, u, N, dt)
    std::list<double> u_old = {9.81,0.0,0.0};
    std::list<double> u_ref = {9.81,0.0,0.0};
    for(int i = 0; i < N; i++){
      // State costs
      // std::list<float> x_ref = nmpc_ref[(ns*i):(ns*i+ns)];
      // #print(x_ref)
      // Setting up itterators
      std::list<double>::iterator Qx_itterator = Qx.begin();
      //std::advance(Qx_itterator, i);
      std::list<double>::iterator x_itterator = x.begin();
      //std::advance(x_itterator, i);
      std::list<double>::iterator x_ref_itterator = nmpc_ref.begin();
      // std::advance(x_ref_itterator, i*ns);
      
      for(int j = 0; j < 8; j++){
        // std::cout << (*Qx_itterator) << ", " <<  (*x_itterator) << ", " << (*x_ref_itterator) << std::endl;
        if(i < 3){
          cost = cost + (*Qx_itterator) * pow((*x_itterator) - (*x_ref_itterator), 2);
        }else{
          cost = cost + (*Qx_itterator) * pow((*x_itterator), 2);
        }
        Qx_itterator++;
        x_itterator++;
        x_ref_itterator++;
      }
      //std::cout << "\n" << std::endl;
      //std::cout << "this is cost: " << cost << std::endl;
      //cost = cost + pow((*Qx_itterator) * (*x_itterator - *x_ref_itterator), 2) + Qx[1]*(x[1]-x_ref[i + 1])**2 + Qx[2]*(x[2]-x_ref[i + 2])**2 + Qx[3]*(x[3]-x_ref[i + 3])**2 + Qx[4]*(x[4]-x_ref[i + 4])**2 + Qx[5]*(x[5]-x_ref[i + 5])**2 + Qx[6]*(x[6]-x_ref[i + 6])**2 + Qx[7]*(x[7]-x_ref[i + 7])**2;  // State weights
      // Input Cost
      
      //Setting up itterators
      std::list<double>::iterator Ru_itterator = Ru.begin();
      std::list<double>::iterator Rd_itterator = Rd.begin();
      //std::list<float>::iterator u_n_itterator = u.begin();
      //std::advance(u_n_itterator, 3 * i);
      std::list<double>::iterator u_ref_itterator = u_ref.begin();
      std::list<double>::iterator u_old_itterator = u_old.begin();
      
      for(int j = 0; j < 3; j++){
        // std::cout << (*Rd_itterator) << ", " <<  u[3*i+j] << ", " << *u_old_itterator << std::endl;
        cost = cost + *Ru_itterator * pow(u[3*i+j] - (*u_ref_itterator), 2);
        cost = cost + *Rd_itterator * pow(u[3*i+j] - *u_old_itterator, 2);
        Ru_itterator++;
        u_old_itterator++;
        Rd_itterator++;
        u_ref_itterator++;
      }
      //std::cout << "this is cost" << cost << std::endl;
      u_old = {u[3*i], u[3*i+1], u[3*i+2]};
      //u_n = u[(3*i):3*i+3];
      //cost += Ru[0]*(u_n[0] - u_ref[0])**2 + Ru[1]*(u_n[1] - u_ref[1])**2 + Ru[2]*(u_n[2] - u_ref[2])**2; // Input weights
      //cost += Rd[0]*(u_n[0] - u_old[0])**2 + Rd[1]*(u_n[1] - u_old[1])**2 + Rd[2]*(u_n[2] - u_old[2])**2; // Input rate weights
      //u_old = u_n;
      // x_hist = x_hist + [x];
      
      x_itterator = x.begin();
      std::list<double>::iterator x2_itterator = x.begin();
      std::advance(x2_itterator, 3);
      //std::advance(u_n_itterator, -2);
      for(int j = 0; j < 3; j++){
        *x_itterator = *x_itterator + dt * *x2_itterator;
        x_itterator++;
        x2_itterator++;
      }
      std::list<double>::iterator x3_itterator = x.begin();
      std::advance(x3_itterator, 7); // x[7]
      *x_itterator = *x_itterator + dt * (sin(*x3_itterator) * cos(*x2_itterator) * u[3*i] - 0.5 * (*x_itterator));
      // std::cout << "THIS IS IMPORTANT: " << *x_itterator << std::endl;
      x_itterator++; // x[4]
      *x_itterator = *x_itterator + dt * (-sin(*x2_itterator) * u[3*i] - 0.5 * (*x_itterator));
      x_itterator++; // x[5]
      // std::cout << "THIS IS IMPORTANT: "<< *x_itterator << ", " << *x3_itterator << ", " << *x2_itterator << ", " << u[3*i] << std::endl;
      *x_itterator = *x_itterator + dt * (cos(*x3_itterator) * cos(*x2_itterator) * u[3*i] - 0.5 * *x_itterator - 9.81);
      x_itterator++;
      //u_n_itterator++;
      *x_itterator = *x_itterator + dt * ((1.0 / 0.8) * (u[3*i + 1] - *x_itterator));
      x_itterator++;
      //u_n_itterator++;
      *x_itterator = *x_itterator + dt * ((1.0 / 0.8) * (u[3*i + 2] - *x_itterator));
      /*
      p_hist = p_hist + [[x[0],x[1],x[2]]];*/
      x_itterator = x.begin();
      p_traj.push_back(*x_itterator);
      x_itterator++;
      p_traj.push_back(*x_itterator);
      x_itterator++;      
      p_traj.push_back(*x_itterator);
      x_itterator++;
      v_traj.push_back(*x_itterator);
      x_itterator++;
      v_traj.push_back(*x_itterator);
      x_itterator++;      
      v_traj.push_back(*x_itterator);
    }
    // std::cout << "\n" << std::endl;
    //int schme = 0;
    // std::cout << "This is p_hist:" << std::endl;
    /*for(std::list<double>::iterator x5_itterator = p_hist.begin(); x5_itterator != p_hist.end(); x5_itterator++){
      if(schme%3 == 0 and schme != 0){
        std::cout << "\n" << std::endl;
      }
      std::cout << *x5_itterator << std::endl;
      schme++;
    }*
    /*schme = 0;
    std::cout << "This is x_hist:\n" << std::endl;
    for(std::list<double>::iterator x4_itterator = x_hist.begin(); x4_itterator != x_hist.end(); x4_itterator++){
      if(schme%8 == 0 and schme != 0){
        std::cout << "\n" << std::endl;
      }
      std::cout << *x4_itterator << std::endl;
      schme++;
    }*/
    // std::cout << cost << std::endl;
    // std::cout << x_hist << std::endl;
    // std::cout << "skibidibap: " << cost << std::endl;
    return std::make_tuple(v_traj, cost, p_traj);
}
