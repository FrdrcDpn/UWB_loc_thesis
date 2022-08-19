% we are simulating with 4 agents 
% importing data for agent 1
% for the case where no inter-agent ranging takes place 
% and for the case where inter-agent ranging does take place

%import data for agent 1 where inter_agent ranging takes place
time_agent_1_dynamic = [];
agent_agent_1_dynamic = [];
xposa_agent_1_dynamic_ground = [];
yposa_agent_1_dynamic_ground = [];
xposa_agent_1_dynamic_est = [];
yposa_agent_1_dynamic_est = [];
zpos_agent_1_dynamic = [];


data_agents_dynamic = importdata('log_2021-10-04-16:09:24.txt') ;
    for(u=1:size(data_agents_dynamic,1))
       if(data_agents_dynamic(u,2)==1)
            time_agent_1_dynamic= [time_agent_1_dynamic,data_agents_dynamic(u,1)];
            agent_agent_1_dynamic= [agent_agent_1_dynamic,data_agents_dynamic(u,2)];
            xposa_agent_1_dynamic_ground= [xposa_agent_1_dynamic_ground,data_agents_dynamic(u,3)];
            yposa_agent_1_dynamic_ground= [yposa_agent_1_dynamic_ground,data_agents_dynamic(u,4)];
            xposa_agent_1_dynamic_est= [xposa_agent_1_dynamic_est,data_agents_dynamic(u,5)];
            yposa_agent_1_dynamic_est= [yposa_agent_1_dynamic_est,data_agents_dynamic(u,6)];
            zpos_agent_1_dynamic= [zpos_agent_1_dynamic,data_agents_dynamic(u,7)];
       end
    end


%import data for agent 1 where no inter_agent ranging takes place
time_agent_1 = [];
agent_agent_1 = [];
xposa_agent_1_ground = [];
yposa_agent_1_ground = [];
xposa_agent_1_est = [];
yposa_agent_1_est = [];
zpos_agent_1 = [];


data_agents = importdata('log_2021-10-05-09:43:02.txt') ;
    for(u=1:size(data_agents,1))
       if(data_agents(u,2)==1)
            time_agent_1= [time_agent_1,data_agents(u,1)];
            agent_agent_1= [agent_agent_1,data_agents(u,2)];
            xposa_agent_1_ground= [xposa_agent_1_ground,data_agents(u,3)];
            yposa_agent_1_ground= [yposa_agent_1_ground,data_agents(u,4)];
            xposa_agent_1_est= [xposa_agent_1_est,data_agents(u,5)];
            yposa_agent_1_est= [yposa_agent_1_est,data_agents(u,6)];
            zpos_agent_1= [zpos_agent_1,data_agents(u,5)];
       end
    end
    
    
subplot(1,2,1) 

plot(xposa_agent_1_dynamic_ground, yposa_agent_1_dynamic_ground, 'Color', 'red');
hold on 
plot(xposa_agent_1_dynamic_est, yposa_agent_1_dynamic_est, 'Color', 'blue');
plot(20,20,'.','MarkerSize',50, 'Color', 'red')
plot(-20,20,'.','MarkerSize',50, 'Color', 'red')
plot(-20,-20,'.','MarkerSize',50, 'Color', 'red')
hold off
xlim([-20 20])
ylim([-20 20])
%rectangle('Position',[-15 -15 30 30])
title('Dynamic beacons')


subplot(1,2,2) 
plot(xposa_agent_1_ground, yposa_agent_1_ground, 'Color', 'red');
hold on 
plot(xposa_agent_1_est, yposa_agent_1_est, 'Color', 'blue');
plot(20,20,'.','MarkerSize',50, 'Color', 'red')
plot(-20,20,'.','MarkerSize',50, 'Color', 'red')
plot(-20,-20,'.','MarkerSize',50, 'Color', 'red')
hold off
xlim([-20 20])
ylim([-20 20])
%rectangle('Position',[-15 -15 30 30])
title('No dynamic beacons') 

sgtitle('Simulation positioning agent nr1 twr in swarm of 10 agents') 


%% calculating rmse of x location
rmse=sqrt(sum((xposa_agent_1_ground(:)-xposa_agent_1_est(:)).^2)/numel(xposa_agent_1_ground));
rmse_dyn=sqrt(sum((xposa_agent_1_dynamic_ground(:)-xposa_agent_1_dynamic_est(:)).^2)/numel(xposa_agent_1_dynamic_ground));

%% plotting error in x and y direction
figure();
subplot(1,2,1) 

plot(time_agent_1_dynamic,abs(xposa_agent_1_dynamic_ground-xposa_agent_1_dynamic_est), 'Color', 'red');
title('Dynamic beacons error x')

subplot(1,2,2) 
plot(time_agent_1_dynamic ,abs(yposa_agent_1_dynamic_ground-yposa_agent_1_dynamic_est), 'Color', 'red');
title('No dynamic beacons error y') 

sgtitle('Simulation positioning agent nr1 twr in swarm of 10 agents') 