
close all force;
clear all;

%% 

loadBag = rosbag("octomap_errt_1.bag");

octomap_selectTopic = select(loadBag,"Topic","/octomap_point_cloud_centers");

pclStruct_errt = readMessages(octomap_selectTopic);

loadBag_2 = rosbag("octomap_errt_2.bag");

octomap_selectTopic_2 = select(loadBag_2,"Topic","/octomap_point_cloud_centers");

pclStruct_errt_2 = readMessages(octomap_selectTopic_2);

loadBag_3 = rosbag("octomap_errt_3.bag");

octomap_selectTopic_3 = select(loadBag_3,"Topic","/octomap_point_cloud_centers");

pclStruct_errt_3 = readMessages(octomap_selectTopic_3);

%% 


loadBag_gbp = rosbag("octomap_gbp_1.bag");

octomap_selectTopic_gbp = select(loadBag_gbp,"Topic","/octomap_point_cloud_centers");

pclStruct_gbp = readMessages(octomap_selectTopic_gbp);

loadBag_gbp_2 = rosbag("octomap_gbp_2.bag");

octomap_selectTopic_gbp_2 = select(loadBag_gbp_2,"Topic","/octomap_point_cloud_centers");

pclStruct_gbp_2 = readMessages(octomap_selectTopic_gbp_2);

loadBag_gbp_3 = rosbag("octomap_gbp_3.bag");

octomap_selectTopic_gbp_3 = select(loadBag_gbp_3,"Topic","/octomap_point_cloud_centers");

pclStruct_gbp_3 = readMessages(octomap_selectTopic_gbp_3);



%%

loadBag_ref = rosbag("octomap_ref_1.bag");

octomap_selectTopic_ref = select(loadBag_ref,"Topic","/octomap_point_cloud_centers");

pclStruct_ref = readMessages(octomap_selectTopic_ref);

loadBag_ref_2 = rosbag("octomap_ref_2.bag");

octomap_selectTopic_ref_2 = select(loadBag_ref_2,"Topic","/octomap_point_cloud_centers");

pclStruct_ref_2 = readMessages(octomap_selectTopic_ref_2);

loadBag_ref_3 = rosbag("octomap_ref_3.bag");

octomap_selectTopic_ref_3 = select(loadBag_ref_3,"Topic","/octomap_point_cloud_centers");

pclStruct_ref_3 = readMessages(octomap_selectTopic_ref_3);


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

for i = 1:1:length(pclStruct_errt_2)
    pcl_2 = readXYZ(pclStruct_errt_2{i});
    pcl_stamped_2 = pclStruct_errt_2{i};
    volume_explored_errt_2(i) = voxel_size^3 * length(pcl_2);
    
    stamp_2(i) = double(pcl_stamped_2.Header.Stamp.Sec);
    stamp_ns_2(i) =  double(pcl_stamped_2.Header.Stamp.Nsec);

    time_errt_2(i) = stamp_2(i) + stamp_ns_2(i)*10^-9;    
end 

for i = 1:1:length(pclStruct_errt_3)
    pcl_3 = readXYZ(pclStruct_errt_3{i});
    pcl_stamped_3 = pclStruct_errt_3{i};
    volume_explored_errt_3(i) = voxel_size^3 * length(pcl_3);

    stamp_3(i) = double(pcl_stamped_3.Header.Stamp.Sec);
    stamp_ns_3(i) =  double(pcl_stamped_3.Header.Stamp.Nsec);

    time_errt_3(i) = stamp_3(i) + stamp_ns_3(i)*10^-9;    
end 


%% 

voxel_size = 0.3;

for i = 1:1:length(pclStruct_gbp)
    pcl_gbp = readXYZ(pclStruct_gbp{i});
    pcl_stamped_gbp = pclStruct_gbp{i};
    volume_explored_gbp(i) = voxel_size^3 * length(pcl_gbp);
    
    stamp_gbp(i) = double(pcl_stamped_gbp.Header.Stamp.Sec);% - 10^9 + Position_r.Header.Stamp.Nsec*10^-9;
    stamp_ns_gbp(i) =  double(pcl_stamped_gbp.Header.Stamp.Nsec);

    time_gbp(i) = stamp_gbp(i) + stamp_ns_gbp(i)*10^-9;    
end 

for i = 1:1:length(pclStruct_gbp_2)
    pcl_gbp_2 = readXYZ(pclStruct_gbp_2{i});
    pcl_stamped_gbp_2 = pclStruct_gbp_2{i};
    volume_explored_gbp_2(i) = voxel_size^3 * length(pcl_gbp_2);
    
    stamp_gbp_2(i) = double(pcl_stamped_gbp_2.Header.Stamp.Sec);
    stamp_ns_gbp_2(i) =  double(pcl_stamped_gbp_2.Header.Stamp.Nsec);

    time_gbp_2(i) = stamp_gbp_2(i) + stamp_ns_gbp_2(i)*10^-9;    
end 

for i = 1:1:length(pclStruct_gbp_3)
    pcl_gbp_3 = readXYZ(pclStruct_gbp_3{i});
    pcl_stamped_gbp_3 = pclStruct_gbp_3{i};
    volume_explored_gbp_3(i) = voxel_size^3 * length(pcl_gbp_3);

    stamp_gbp_3(i) = double(pcl_stamped_gbp_3.Header.Stamp.Sec);
    stamp_ns_gbp_3(i) =  double(pcl_stamped_gbp_3.Header.Stamp.Nsec);

    time_gbp_3(i) = stamp_gbp_3(i) + stamp_ns_gbp_3(i)*10^-9;    
end 


%%


voxel_size = 0.3;

for i = 1:1:length(pclStruct_ref)
    pcl_ref = readXYZ(pclStruct_ref{i});
    pcl_stamped_ref = pclStruct_ref{i};
    volume_explored_ref(i) = voxel_size^3 * length(pcl_ref);
    
    stamp_ref(i) = double(pcl_stamped_ref.Header.Stamp.Sec);% - 10^9 + Position_r.Header.Stamp.Nsec*10^-9;
    stamp_ns_ref(i) =  double(pcl_stamped_ref.Header.Stamp.Nsec);

    time_ref(i) = stamp_ref(i) + stamp_ns_ref(i)*10^-9;    
end 

for i = 1:1:length(pclStruct_ref_2)
    pcl_ref_2 = readXYZ(pclStruct_ref_2{i});
    pcl_stamped_ref_2 = pclStruct_ref_2{i};
    volume_explored_ref_2(i) = voxel_size^3 * length(pcl_ref_2);
    
    stamp_ref_2(i) = double(pcl_stamped_ref_2.Header.Stamp.Sec);
    stamp_ns_ref_2(i) =  double(pcl_stamped_ref_2.Header.Stamp.Nsec);

    time_ref_2(i) = stamp_ref_2(i) + stamp_ns_ref_2(i)*10^-9;    
end 

for i = 1:1:length(pclStruct_ref_3)
    pcl_ref_3 = readXYZ(pclStruct_ref_3{i});
    pcl_stamped_ref_3 = pclStruct_ref_3{i};
    volume_explored_ref_3(i) = voxel_size^3 * length(pcl_ref_3);

    stamp_ref_3(i) = double(pcl_stamped_ref_3.Header.Stamp.Sec);
    stamp_ns_ref_3(i) =  double(pcl_stamped_ref_3.Header.Stamp.Nsec);

    time_ref_3(i) = stamp_ref_3(i) + stamp_ns_ref_3(i)*10^-9;    
end 


%%
% PLOT SECTION

afigure(1)

plot(time_errt - 14, volume_explored_errt, 'LineWidth',4.0, 'Color',[0,0.8,0])
hold on
plot(time_errt_2 - 743, volume_explored_errt_2, 'LineWidth',4.0, 'Color',[0,0.8,0])
hold on
plot(time_errt_3 -2328, volume_explored_errt_3, 'LineWidth',4.0, 'Color',[0,0.8,0])
 hold on 
plot(time_gbp - 2114, volume_explored_gbp, 'LineWidth',4.0, 'Color',[0,0,0.8])
hold on
plot(time_gbp_2 - 5991, volume_explored_gbp_2, 'LineWidth',4.0, 'Color',[0,0,0.8])
hold on
plot(time_gbp_3 -7304, volume_explored_gbp_3, 'LineWidth',4.0, 'Color',[0,0,0.8])
 hold on 
plot(time_ref - 442, 2*volume_explored_ref, 'LineWidth',4.0, 'Color',[0.8,0,0])
hold on
plot(time_ref_2 - 1265, 2*volume_explored_ref_2, 'LineWidth',4.0, 'Color',[0.8,0,0])
hold on
plot(time_ref_3 -1844, 2*volume_explored_ref_3, 'LineWidth',4.0, 'Color',[0.8,0,0])


xlabel('Time (s)','Interpreter','latex');
ylabel('Explored Volume [$m^{3}$]', 'Interpreter','latex')
xlim([0 800]);

ax = gca;
width=900;
height=400;
set(gcf,'position',[10,10,width,height]);
set(gcf,'color','white');
legend('ERRT 1', 'ERRT 2', 'ERRT 3', 'GB Planner 1', 'GB Planner 2', 'GB Planner 3', 'Frontiers [REF] 1', 'Frontiers [REF] 2', 'Frontiers [REF] 3', 'Interpreter','latex','Fontsize',20);
%legend('ERRT','GB Planner 2','Interpreter','latex','Fontsize',20);

%xlim([0 250])
%legend('V$^{l} = 15m$, n$_{traj} = 60$, size(N) = 1000', 'V$^{l} = 20m$, n$_{traj} = 60$, size(N) = 2000', 'V$^{l} = 25m$, n$_{traj} = 80$, size(N) = 3000','Interpreter','latex','Fontsize',20);
%legend('n$_{traj} = 100$, V$^{l} = 15m$, size(N) = 2000', 'n$_{traj} = 80$, \ V$^{l} = 15m$, size(N) = 2000', 'n$_{traj} = 60$, \ V$^{l} = 15m$, size(N) = 2000','Interpreter','latex','Fontsize',20);
%legend('ERRT', 'GB Planner 2', 'Frontiers (REF)','Interpreter','latex','Fontsize',20);
