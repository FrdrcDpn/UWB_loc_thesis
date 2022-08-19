

%import data for agent 1 where no inter_agent ranging takes place
time_agent = [];
agent_agent= [];
xposa_agent = [];
yposa_agent = [];
zpos_agent = [];

totaldatax = [];
totaldatay = [];
data_agents = importdata('log_2021-10-04-15:44:24.txt') ;

for(k=1:16)
    xposa_agent = [];
yposa_agent = [];
    for(u=1:size(data_agents,1))
       if(data_agents(u,2)==k)
            time_agent= [time_agent,data_agents(u,1)];
            agent_agent= [agent_agent,data_agents(u,2)];
            xposa_agent= [xposa_agent,data_agents(u,3)];
            yposa_agent= [yposa_agent,data_agents(u,4)];
            zpos_agent= [zpos_agent,data_agents(u,5)];
       end
    end
    xposa_agent= xposa_agent((length(xposa_agent)-50):length(xposa_agent));
    yposa_agent= yposa_agent((length(yposa_agent)-50):length(yposa_agent));
    plot(yposa_agent(1,:), xposa_agent(1,:), 'Color', 'red');
    hold on 
    totaldatax= [totaldatax, xposa_agent];
    totaldatay= [totaldatay, yposa_agent];
end





hold on 
plot(-20,0,'.','MarkerSize',50, 'Color', 'red')
plot(15,15,'.','MarkerSize',5, 'Color', 'blue')
plot(-15,-15,'.','MarkerSize',5, 'Color', 'blue')
plot(-15,15,'.','MarkerSize',5, 'Color', 'blue')
plot(15,-15,'.','MarkerSize',5, 'Color', 'blue')

plot(5,5,'.','MarkerSize',5, 'Color', 'blue')
plot(-5,-5,'.','MarkerSize',5, 'Color', 'blue')
plot(-5,5,'.','MarkerSize',5, 'Color', 'blue')
plot(5,-5,'.','MarkerSize',5, 'Color', 'blue')

plot(5,15,'.','MarkerSize',5, 'Color', 'blue')
plot(-5,-15,'.','MarkerSize',5, 'Color', 'blue')
plot(-5,15,'.','MarkerSize',5, 'Color', 'blue')
plot(5,-15,'.','MarkerSize',5, 'Color', 'blue')

plot(15,5,'.','MarkerSize',5, 'Color', 'blue')
plot(-15,-5,'.','MarkerSize',5, 'Color', 'blue')
plot(-15,5,'.','MarkerSize',5, 'Color', 'blue')
plot(15,-5,'.','MarkerSize',5, 'Color', 'blue')

plot(-20,-20,'.','MarkerSize',50, 'Color', 'red')
plot(-20,20,'.','MarkerSize',50, 'Color', 'red')
hold off
xlim([-20 20])
ylim([-20 20])


sgtitle('plotter 16 agents') 

