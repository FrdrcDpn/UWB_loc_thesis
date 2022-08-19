% we are simulating with 4 agents 
% importing data for agent 1
% for the case where no inter-agent ranging takes place 
% and for the case where inter-agent ranging does take place

%import data for agent 1 where inter_agent ranging takes place
time_agent_1_dynamic = [];
agent_agent_1_dynamic = [];
xposa_agent_1_dynamic = [];
yposa_agent_1_dynamic = [];
zpos_agent_1_dynamic = [];
%data_agents_dynamic = importdata('log_2021-09-21-13:14:51.txt') ;

data_agents_dynamic = importdata('log_2021-09-21-13:31:01.txt') ;
    for(u=1:size(data_agents_dynamic,1))
       if(data_agents_dynamic(u,2)==1)
            time_agent_1_dynamic= [time_agent_1_dynamic,data_agents_dynamic(u,1)];
            agent_agent_1_dynamic= [agent_agent_1_dynamic,data_agents_dynamic(u,2)];
            xposa_agent_1_dynamic= [xposa_agent_1_dynamic,data_agents_dynamic(u,3)];
            yposa_agent_1_dynamic= [yposa_agent_1_dynamic,data_agents_dynamic(u,4)];
            zpos_agent_1_dynamic= [zpos_agent_1_dynamic,data_agents_dynamic(u,5)];
       end
    end


%import data for agent 1 where no inter_agent ranging takes place
time_agent_1 = [];
agent_agent_1 = [];
xposa_agent_1 = [];
yposa_agent_1 = [];
zpos_agent_1 = [];


data_agents = importdata('log_2021-09-21-13:30:02.txt') ;
    for(u=1:size(data_agents,1))
       if(data_agents(u,2)==1)
            time_agent_1= [time_agent_1,data_agents(u,1)];
            agent_agent_1= [agent_agent_1,data_agents(u,2)];
            xposa_agent_1= [xposa_agent_1,data_agents(u,3)];
            yposa_agent_1= [yposa_agent_1,data_agents(u,4)];
            zpos_agent_1= [zpos_agent_1,data_agents(u,5)];
       end
    end
    
    
subplot(1,2,1) 
plot(xposa_agent_1_dynamic, yposa_agent_1_dynamic, 'Color', 'red');
hold on 
plot(20,20,'.','MarkerSize',50, 'Color', 'red')
plot(-20,20,'.','MarkerSize',50, 'Color', 'red')
plot(-20,-20,'.','MarkerSize',50, 'Color', 'red')
hold off
xlim([-20 20])
ylim([-20 20])
rectangle('Position',[-17.5 -17.5 35 35])
title('Dynamic beacons')


subplot(1,2,2) 
plot(xposa_agent_1, yposa_agent_1, 'Color', 'red');
hold on 
plot(20,20,'.','MarkerSize',50, 'Color', 'red')
plot(-20,20,'.','MarkerSize',50, 'Color', 'red')
plot(-20,-20,'.','MarkerSize',50, 'Color', 'red')
hold off
xlim([-20 20])
ylim([-20 20])
rectangle('Position',[-17.5 -17.5 35 35])
title('No dynamic beacons') 

sgtitle('Simulation positioning agent nr1 tdoa in swarm of 10 agents') 

