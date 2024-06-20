

close all force;
clear all;

%% 


loadBag = rosbag("octomap_t7.bag");

csa_selectTopic = select(loadBag,"Topic","/errt_execution_time");

csa_readMsg = readMessages(csa_selectTopic,"DataFormat", "struct");

for k = 1:length(csa_readMsg)

    execution_time(k) = csa_readMsg{k}.Data(1);
    
end


for t = 1:length(csa_readMsg)
    time(t) = t;
end

%% 

loadBag_2 = rosbag("octomap_t8.bag");

csa_selectTopic_2 = select(loadBag_2,"Topic","/errt_execution_time");

csa_readMsg_2 = readMessages(csa_selectTopic_2,"DataFormat", "struct");


for k_2 = 1:length(csa_readMsg_2)

    execution_time_2(k_2) = csa_readMsg_2{k_2}.Data(1);
    
end


for t_2 = 1:length(csa_readMsg_2)
    time_2(t_2) = t_2;
end


%%


loadBag_3 = rosbag("octomap_t1.bag");

csa_selectTopic_3 = select(loadBag_3,"Topic","/errt_execution_time");

csa_readMsg_3 = readMessages(csa_selectTopic_3,"DataFormat", "struct");


for k_3 = 1:length(csa_readMsg_3)

    execution_time_3(k_3) = csa_readMsg_3{k_3}.Data(1);
    
end


for t_3 = 1:length(csa_readMsg_3)
    time_3(t_3) = t_3;
end


%% 

% PLOT SECTION

afigure(1)
plot(time, 0.001*execution_time, 'LineWidth',4.0,'Color',[1,0,0]);
xlabel('Iteration','Interpreter','latex');
ylabel('Execution Time [$s$]', 'Interpreter','latex')
% ylabel off
%xlim([0 871])
%yticks([0:200:1000]);
grid off
ax = gca;
%ax.Clipping = 'off';

width=900;
height=400;
set(gcf,'position',[10,10,width,height]);
set(gcf,'color','white');
hold on
plot(time_2, 0.001*execution_time_2, 'LineWidth',4.0,'Color',[0,0.6,0]);

hold on 

plot(time_3, 0.001*execution_time_3, 'LineWidth',4.0,'Color',[0,0,0.6]);

%legend('Evaluate$_{traj}$', 'Evaluate$_{goal}$','Interpreter','latex','Fontsize',20);

legend('T2', 'T1', 'T3','Interpreter','latex','Fontsize',20);


%%

afigure(2)

% bar(0.001*execution_time_3);
% hold on 
%bar(0.001*execution_time_2);
%hold on 
bar(0.001*execution_time);
xlim([0 25.5])
title('Hospital Environment','Interpreter','latex')
xlabel('Iteration','Interpreter','latex');
ylabel('Execution Time [$s$]', 'Interpreter','latex')
legend('V$^{l} = 20m$, n$_{traj} = 60$, size(N) = 2000', 'V$^{l} = 10m$, n$_{traj} = 60$, size(N) = 2000','Interpreter','latex','Fontsize',20);
%legend('n$_{traj} = 100$, V$^{l} = 15m$, size(N) = 2000', 'n$_{traj} = 80$, \ V$^{l} = 15m$, size(N) = 2000', 'n$_{traj} = 60$, \ V$^{l} = 15m$, size(N) = 2000','Interpreter','latex','Fontsize',20);
%legend('Evaluate Trajectory','Evaluate Goal','Interpreter','latex','Fontsize',20);