%% Importing variables from simulation file 

%import data for agent 1 where no inter_agent ranging takes place
time_agent_1 = [];
agent_agent_1 = [];
xposa_agent_1_ground = [];
yposa_agent_1_ground = [];
xposa_agent_1_est = [];
yposa_agent_1_est = [];
xposa_agent_1_traj = [];
yposa_agent_1_traj = [];
zpos_agent_1 = [];
c1 = [];
c2 = [];
c3 = [];
c4 = [];
c5 = [];
c6 = [];

contr_desx = [];
contr_desy = [];
totalposa_agent_1_ground = [];
totalposa_agent_1_est = [];
totalposa_agent_1_traj = [];
contr_desz = [];
contr_x = [];
contr_y = [];
contr_z = [];

contr_theta = [];
contr_phi = [];
contr_psi = [];


data_agents = importdata('log_2022-06-14-15:08:44.txt') ;
    for(u=1:size(data_agents,1))
       if(data_agents(u,2)==1) 
            time_agent_1= [time_agent_1,data_agents(u,1)];
            agent_agent_1= [agent_agent_1,data_agents(u,2)];
            xposa_agent_1_ground= [xposa_agent_1_ground,data_agents(u,3)];
            yposa_agent_1_ground= [yposa_agent_1_ground,data_agents(u,4)];
            xposa_agent_1_est= [xposa_agent_1_est,data_agents(u,5)];
            yposa_agent_1_est= [yposa_agent_1_est,data_agents(u,6)];
            xposa_agent_1_traj= [xposa_agent_1_traj,data_agents(u,7)];
            yposa_agent_1_traj= [yposa_agent_1_traj,data_agents(u,8)];
            
            contr_desx= [contr_desx,data_agents(u,9)];
            contr_desy= [contr_desy,data_agents(u,10)];
            contr_desz= [contr_desz,data_agents(u,11)];
            
            contr_x= [contr_x,data_agents(u,12)];
            contr_y= [contr_y,data_agents(u,13)];
            contr_z= [contr_z,data_agents(u,14)];
            
            contr_theta= [contr_theta,data_agents(u,15)];
            contr_phi= [contr_phi,data_agents(u,16)];
            contr_psi= [contr_psi,data_agents(u,17)];
            
            c1 = [c1, data_agents(u,18)];
            c2 = [c2, data_agents(u,19)];
            c3 = [c3, data_agents(u,20)];
            c4 = [c4, data_agents(u,21)];
            c5 = [c5, data_agents(u,22)];
            c6 = [c6, data_agents(u,23)];
       end
    end
%% Generating samples from our state estimate + covariance matrix

% Gaussian mean and covariance position estimate

additional_noise = [];
calculated_additional_noise = [];
additional_noise_x = [];
calculated_additional_noise_x = [];
additional_noise_y = [];
calculated_additional_noise_y = [];

figure(1); 
for (i = 1:40)
    d = 2; % number of dimensions
    datapoint_simulation = i; % selected datapoint from simulation log
    num = 10000; % number of samples from covariance matrix distribution

    % take our state estimate as mu
    mu = [yposa_agent_1_est(datapoint_simulation) xposa_agent_1_est(datapoint_simulation)];

    % build the covariance matrix from the simulation
    sigma = [c1(datapoint_simulation) c2(datapoint_simulation); c4(datapoint_simulation) c3(datapoint_simulation)];
    sigma_def = (sigma + sigma.') / 2;
    %generate the n samples from the distribution
    
    h(i+3) = error_ellipse(sigma,mu); 
    set(h(i+3), 'Color', [.5 .5 .5])
    hold on
    
end

h(1) = plot(yposa_agent_1_ground(1:40), xposa_agent_1_ground(1:40), 'Color', 'red','DisplayName','ground truth');
hold on 
h(2) = plot(yposa_agent_1_est(1:40), xposa_agent_1_est(1:40), 'Color', 'blue','DisplayName','state-estimate');
hold on 
h(3) = plot(yposa_agent_1_traj(1:40), xposa_agent_1_traj(1:40), 'Color', 'green','DisplayName','trajectory');
set(h(4), 'DisplayName', 'Error ellipses')
lgd = legend(h(1:4));
lgd.NumColumns = 1;
xlim([-10 8])
ylim([-10 5])
hold off

xlabel('X position [m]')
ylabel('Y position [m]')
title('Covariance error ellipse')
matlab2tikz('Error_ellipse.tex');
