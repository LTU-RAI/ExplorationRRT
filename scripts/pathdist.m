close all, clear all, clc
%rosbag info '2019-09-03-10-12-00.bag'
bag = rosbag('test4_rondell_back_2023-04-05-11-07-56.bag');
%%
path_topic = select(bag, 'Topic', '/shafter2/lio_sam/mapping/path');
%odom_r = select(bag, 'Topic', 'shafter3d/odom/sample');
%ball = select(bag, 'Topic', '/crazyflie/vicon/ball/ball/odom');
%input = select(bag, 'Topic', '/crazyflie3/cmd_vel');
%hoop = select(bag, 'Topic', '
%%
msgStructs = readMessages(path_topic,'DataFormat','struct');
%%
path = msgStructs{length(msgStructs)}.Poses
distsum = 0
for i = 2:1:length(path)
    p_prev = path(i-1).Pose.Position 
    p  =  path(i).Pose.Position
    position(i) = path(i).Pose.Position
    x = p.X
    y = p.Y
    z = p.Z
    x1 = p_prev.X
    y1 = p_prev.Y
    z1 = p_prev.Z
    dist = sqrt((x-x1)^2 + (y-y1)^2 + (z-z1)^2)
    distsum = distsum + dist 
    
    
end 
%%
close all, clear all, clc
%%
%rosbag info '2019-09-03-10-12-00.bag'
bag_errt = rosbag('GOOD_errt_darpa_final_octomap_2023-05-03-11-16-33.bag');
%bag_gb = rosbag('gbp_large_cave_octomap_bag_2023-04-25-16-33-04.bag');
%bag_errt = rosbag('errt_large_cave_octomap_bag_2023-04-25-16-27-53.bag');
%bag_errt = rosbag('errt_narrow_cave_octomap_bag_2023-04-25-16-30-28.bag');
%bag_errt = rosbag('errt_darpa_final_octomap.bag');


%pcl_topic_gb = select(bag_gb, 'Topic','/octomap_point_cloud_centers');
%odom_topic_gb = select(bag_gb, 'Topic','/rmf_obelix/ground_truth/odometry');

pcl_topic_errt = select(bag_errt, 'Topic','/octomap_point_cloud_centers');
odom_topic_errt = select(bag_errt, 'Topic','/hummingbird/ground_truth/odometry');
%odom_topic_errt = select(bag_errt, 'Topic','/shafter2/odometry/imu');


%pclStruct_gb = readMessages(pcl_topic_gb);
%odomStruct_gb = readMessages(odom_topic_gb,'DataFormat','struct');

pclStruct_errt = readMessages(pcl_topic_errt);
odomStruct_errt = readMessages(odom_topic_errt,'DataFormat','struct');


%%
voxel_size = 0.3
% for i = 1:1:length(pclStruct_gb)
%    pcl = readXYZ(pclStruct_gb{i});
%    pcl_stamped = pclStruct_gb{i};
%    volume_explored_gb_3(i) = voxel_size^3 * length(pcl);
%    
%    stamp(i) = double(pcl_stamped.Header.Stamp.Sec);% - 10^9 + Position_r.Header.Stamp.Nsec*10^-9;
%    stamp_ns(i) =  double(pcl_stamped.Header.Stamp.Nsec);
% 
%    time_gb_3(i) = stamp(i) + stamp_ns(i)*10^-9;    
% end 

for i = 1:1:length(pclStruct_errt)
    pcl = readXYZ(pclStruct_errt{i});
    pcl_stamped = pclStruct_errt{i};
    volume_explored_errt_2(i) = voxel_size^3 * length(pcl);
    
    stamp(i) = double(pcl_stamped.Header.Stamp.Sec);% - 10^9 + Position_r.Header.Stamp.Nsec*10^-9;
    stamp_ns(i) =  double(pcl_stamped.Header.Stamp.Nsec);

    time_errt_2(i) = stamp(i) + stamp_ns(i)*10^-9;    
end 

d_sum = 0;
path_len = 0;
velocity_mag = 0;
for i = 2:1:length(odomStruct_errt)
    odom = odomStruct_errt{i};
    odom_prev = odomStruct_errt{i-1};

    p_prev = [odom_prev.Pose.Pose.Position.X, odom_prev.Pose.Pose.Position.Y, odom_prev.Pose.Pose.Position.Z];
    p = [odom.Pose.Pose.Position.X, odom.Pose.Pose.Position.Y, odom.Pose.Pose.Position.Z];
    dist = sqrt((p(1) - p_prev(1))^2 + (p(2) - p_prev(2))^2 + (p(3) - p_prev(3))^2 );
    d_sum = d_sum + dist;
    path_len(i) = d_sum;
    velocity_mag(i) = sqrt( (odom.Twist.Twist.Linear.X)^2 + (odom.Twist.Twist.Linear.Y)^2 + (odom.Twist.Twist.Linear.Z)^2);
    stamp(i) = double(odom.Header.Stamp.Sec);% - 10^9 + Position_r.Header.Stamp.Nsec*10^-9;
    stamp_ns(i) =  double(odom.Header.Stamp.Nsec);

    time_errt_odom_2(i) = stamp(i) + stamp_ns(i)*10^-9;  
end 
mean_vel_errt = mean(velocity_mag)

% for i = 2:1:length(odomStruct_gb)
%     odom = odomStruct_gb{i};
%     odom_prev = odomStruct_gb{i-1};
% 
%     p_prev = [odom_prev.Pose.Pose.Position.X, odom_prev.Pose.Pose.Position.Y, odom_prev.Pose.Pose.Position.Z];
%     p = [odom.Pose.Pose.Position.X, odom.Pose.Pose.Position.Y, odom.Pose.Pose.Position.Z];
%     dist = sqrt((p(1) - p_prev(1))^2 + (p(2) - p_prev(2))^2 + (p(3) - p_prev(3))^2 );
%     d_sum = d_sum + dist;
%     path_len(i) = d_sum;
%     velocity_mag(i) = sqrt( (odom.Twist.Twist.Linear.X)^2 + (odom.Twist.Twist.Linear.Y)^2 + (odom.Twist.Twist.Linear.Z)^2);
%     stamp(i) = double(odom.Header.Stamp.Sec);% - 10^9 + Position_r.Header.Stamp.Nsec*10^-9;
%     stamp_ns(i) =  double(odom.Header.Stamp.Nsec);
% 
%     time_gb_odom_3(i) = stamp(i) + stamp_ns(i)*10^-9;  
% end 
% mean_vel_gb = mean(velocity_mag)

% index = round(length(odomStruct_gb) / length(pclStruct_gb));
% for i = 1:1:length(pclStruct_gb)
%     path_len_sample_gb_3(i) = path_len(i*(index-1));
% end 


index = round(length(odomStruct_errt) / length(pclStruct_errt));
for i = 2:1:length(pclStruct_errt)
    path_len_sample_errt_2(i) = path_len(i*(index-1));
end

% %%
% afigure()
% hold on
% %plot(time_gb, volume_explored_gb)
% plot(time_errt, volume_explored_errt)
% %legend('GBPlanner', 'ERRT')
% xlabel('Time(s)')
% ylabel('Volume Explored(m³)')
% %ylim([0,5])

 
%%
afigure()
hold on
plot(path_len_sample_gb, volume_explored_gb,'k', path_len_sample_gb_2, volume_explored_gb_2,'k', path_len_sample_gb_3, volume_explored_gb_3,'k')
plot(path_len_sample_errt, volume_explored_errt, path_len_sample_errt_2, volume_explored_errt_2)

xlabel('Total Exploration Distance (m)')
ylabel('Explored Volume (m^3)')
legend('GBPlanner')
title('DARPA Final Stage World')
