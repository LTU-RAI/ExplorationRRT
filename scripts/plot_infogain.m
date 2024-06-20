

close all force;
clear all;

%% 

loadBag = rosbag("octomap_t7.bag");

octomap_selectTopic = select(loadBag,"Topic","/octomap_point_cloud_centers");

pclStruct_errt = readMessages(octomap_selectTopic);

loadBag_2 = rosbag("octomap_t8.bag");

octomap_selectTopic_2 = select(loadBag_2,"Topic","/octomap_point_cloud_centers");

pclStruct_errt_2 = readMessages(octomap_selectTopic_2);

loadBag_3 = rosbag("octomap_t6.bag");

octomap_selectTopic_3 = select(loadBag_3,"Topic","/octomap_point_cloud_centers");

pclStruct_errt_3 = readMessages(octomap_selectTopic_3);

%% 

voxel_size = 0.3;

for i = 1:1:length(pclStruct_errt)
    pcl = readXYZ(pclStruct_errt{i});
    pcl_stamped = pclStruct_errt{i};
    volume_explored_errt(i) = voxel_size^3 * length(pcl);
    
    stamp(i) = double(pcl_stamped.Header.Stamp.Sec);% - 10^9 + Position_r.Header.Stamp.Nsec*10^-9;
    stamp_ns(i) =  double(pcl_stamped.Header.Stamp.Nsec);

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


voxel_size = 0.3;

for i = 1:1:length(pclStruct_errt_3)
    pcl_3 = readXYZ(pclStruct_errt_3{i});
    pcl_stamped_3 = pclStruct_errt_3{i};
    volume_explored_errt_3(i) = voxel_size^3 * length(pcl_3);
    
    stamp_3(i) = double(pcl_stamped_3.Header.Stamp.Sec);
    stamp_ns_3(i) =  double(pcl_stamped_3.Header.Stamp.Nsec);

    time_errt_3(i) = stamp_3(i) + stamp_ns_3(i)*10^-9;    
end 


%% 

% PLOT SECTION

afigure(1)
%plot(time, 0.001*execution_time, 'LineWidth',4.0,'Color',[1,0,0]);

plot(time_errt, volume_explored_errt, 'LineWidth',4.0)
hold on
plot(time_errt_2 , volume_explored_errt_2, 'LineWidth',4.0)
%hold on
%plot(time_errt , volume_explored_errt, 'LineWidth',4.0)

xlabel('Time (s)','Interpreter','latex');
ylabel('Explored Volume [$m^{3}$]', 'Interpreter','latex')
% ylabel off
%xlim([0 871])
%yticks([0:200:1000]);

ax = gca;
%ax.Clipping = 'off';

width=900;
height=400;
set(gcf,'position',[10,10,width,height]);
set(gcf,'color','white');
hold on


xlim([0 250])
%legend('V$^{l} = 15m$, n$_{traj} = 60$, size(N) = 1000', 'V$^{l} = 20m$, n$_{traj} = 60$, size(N) = 2000', 'V$^{l} = 25m$, n$_{traj} = 80$, size(N) = 3000','Interpreter','latex','Fontsize',20);

legend('n$_{traj} = 100$, V$^{l} = 15m$, size(N) = 2000', 'n$_{traj} = 80$, \ V$^{l} = 15m$, size(N) = 2000', 'n$_{traj} = 60$, \ V$^{l} = 15m$, size(N) = 2000','Interpreter','latex','Fontsize',20);
