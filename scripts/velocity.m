
close all force;
clear all;

%% 

loadBag = rosbag("octomap_t1.bag");

odom_selectTopic = select(loadBag,"Topic","/hummingbird/ground_truth/odometry");

Struct_errt = readMessages(odom_selectTopic);

loadBag_2 = rosbag("octomap_t2.bag");

odom_selectTopic_2 = select(loadBag_2,"Topic","/hummingbird/ground_truth/odometry");

Struct_errt_2 = readMessages(odom_selectTopic_2);
    

%% 

voxel_size = 0.3;

for i = 1:1:length(Struct_errt)

    vx_errt(i) = Struct_errt{i}.Twist.Twist.Linear.X;
    vy_errt(i) = Struct_errt{i}.Twist.Twist.Linear.Y;
    vz_errt(i) = Struct_errt{i}.Twist.Twist.Linear.Z;
    
    v_mod(i) = sqrt((vx_errt(i) * vx_errt(i)) + (vy_errt(i) * vy_errt(i)) + (vz_errt(i) * vz_errt(i)));

    stamp(i) = double(Struct_errt{i}.Header.Stamp.Sec);% - 10^9 + Position_r.Header.Stamp.Nsec*10^-9;
    stamp_ns(i) =  double(Struct_errt{i}.Header.Stamp.Nsec);

    time_errt(i) = stamp(i) + stamp_ns(i)*10^-9;    
end 

%% 


voxel_size = 0.3;

for i = 1:1:length(pclStruct_errt_2)
    pcl_2 = readXYZ(pclStruct_errt_2{i});
    pcl_stamped_2 = pclStruct_errt_2{i};
    volume_explored_errt_2(i) = voxel_size^3 * length(pcl_2);
    
    stamp_2(i) = double(pcl_stamped_2.Header.Stamp.Sec);
    stamp_ns_2(i) =  double(pcl_stamped_2.Header.Stamp.Nsec);

    time_errt_2(i) = stamp_2(i) + stamp_ns_2(i)*10^-9;    
end 



%% 

% PLOT SECTION

afigure(1)
%plot(time, 0.001*execution_time, 'LineWidth',4.0,'Color',[1,0,0]);

%plot(time_errt, movmean(v_mod, 10), 'LineWidth',4.0)
h = histogram(v_mod);
h.FaceColor = [0.8 0 0];
% hold on
% plot(time_errt_2 , volume_explored_errt_2, 'LineWidth',4.0)

xlabel('velocity ($m/s$)','Interpreter','latex');
ylabel('Instances', 'Interpreter','latex')
% ylabel off
xlim([0 1.6])
%yticks([0:200:1000]);

ax = gca;
%ax.Clipping = 'off';

width=900;
height=400;
set(gcf,'position',[10,10,width,height]);
set(gcf,'color','white');



%xlim([0 800])
%legend('V$^{l} = 15m$, n$_{traj} = 60$, size(N) = 1000', 'V$^{l} = 20m$, n$_{traj} = 60$, size(N) = 2000', 'V$^{l} = 25m$, n$_{traj} = 80$, size(N) = 3000','Interpreter','latex','Fontsize',20);

%legend('','Interpreter','latex','Fontsize',20);
