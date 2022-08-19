%% Importing variables from simulation file 
clear all
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


data_agents = importdata('log_2022-06-14-16:15:13.txt') ;
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
corr = [];
P =  [];

diff_x = [];
diff_y= [];
 diff_final = [];
 var_final = [];
 econsist = [];
for (i = 1:length(xposa_agent_1_est))
    datapoint_simulation = i; 
    dif_x = xposa_agent_1_est(datapoint_simulation) - xposa_agent_1_ground(datapoint_simulation);
    dif_y = yposa_agent_1_est(datapoint_simulation) - yposa_agent_1_ground(datapoint_simulation);
    d_final_est = sqrt(xposa_agent_1_est(datapoint_simulation)*xposa_agent_1_est(datapoint_simulation) +yposa_agent_1_est(datapoint_simulation)*yposa_agent_1_est(datapoint_simulation)) ;
    d_final_ground = sqrt(xposa_agent_1_ground(datapoint_simulation)*xposa_agent_1_ground(datapoint_simulation) + yposa_agent_1_ground(datapoint_simulation)*yposa_agent_1_ground(datapoint_simulation));
    diff_x = [diff_x,dif_x];
    diff_y = [diff_y,dif_y];
    diff_final = [diff_final,d_final_est-d_final_ground];
    d = 2; % number of dimensions 
    datapoint_simulation = i; % selected datapoint from simulation log
    num = 10000; % number of samples from covariance matrix distribution

    % take our state estimate as mu
    mu = [xposa_agent_1_est(datapoint_simulation) yposa_agent_1_est(datapoint_simulation)];

    % build the covariance matrix from the simulation
    sigma = [c1(datapoint_simulation) c2(datapoint_simulation); c4(datapoint_simulation) c3(datapoint_simulation)];
    X_est = [(xposa_agent_1_est(datapoint_simulation)-xposa_agent_1_ground(datapoint_simulation))/sqrt(xposa_agent_1_est(datapoint_simulation)*xposa_agent_1_est(datapoint_simulation)+xposa_agent_1_ground(datapoint_simulation)*xposa_agent_1_ground(datapoint_simulation))  ;  (yposa_agent_1_est(datapoint_simulation)-yposa_agent_1_ground(datapoint_simulation))/sqrt(yposa_agent_1_est(datapoint_simulation)*yposa_agent_1_est(datapoint_simulation)+yposa_agent_1_ground(datapoint_simulation)*yposa_agent_1_ground(datapoint_simulation))];
   
    econsist = transpose(X_est)*sigma*(X_est);
    sigma_def = (sigma + sigma.') / 2;
    %generate the n samples from the distribution
    X = mvnrnd(mu, sigma_def, num);
    
 

    %% generate our twr measurements with gaussian noise

    mu = 0; % 0.1 for twr measurements
    sigma = 0.16; %std

    d = sqrt(X(:,1).*X(:,1) + X(:,2).*X(:,2)); % generate distance measurement from origin

    % generate noisy distance measurements
    d_noisy = [];
  
    for k = 1:length(d) 
    d_noisy(k) = d(k) +  normrnd(mu,sigma);
    end 
    
    % the standard deviation of our ranging measurements
    sdt_add_sim = std(d_noisy);
    
    % now calculate our additive twr noise due to position uncertainty for
    % comparison
    
    % the angle that our quadrotor makes wrt. the origin
    theta = atan((yposa_agent_1_est(datapoint_simulation))/(xposa_agent_1_est(datapoint_simulation)));
    P(:,1) = d_noisy*cos(theta);
    P(:,2) = d_noisy*sin(theta);
    %% calculations for std in direction or measurement
    
    for k = 1:length(d) 
    L(k) = d(k);
    end 
    covo = corrcoef(L,d_noisy);
    corr = [corr, covo(1,2)];
    
    % calculate std 
    variance_add = cos(theta)*cos(theta)*c1(datapoint_simulation) + sin(theta)*sin(theta)*c3(datapoint_simulation) +2*c2(datapoint_simulation)*sin(theta)*cos(theta); 
    variance_total_add = abs(variance_add);
    var_final = [var_final , variance_total_add];
    std_add = sqrt(0.16*0.16 + variance_total_add );%+ 2*0.16*sqrt(variance_total_add)*covo(1,2));
    
    % write to df
    calculated_additional_noise = [calculated_additional_noise,std_add];
    additional_noise = [additional_noise,sdt_add_sim];
    
    

    
    %% calculations for std in x and y direction 
    
    % calculate std 
    variance_add_x = c1(datapoint_simulation); 
    variance_add_y = c3(datapoint_simulation); 
    std_add_x = sqrt(0.025*0.025*cos(theta)*cos(theta) + variance_add_x);
    std_add_y = sqrt(0.025*0.025*sin(theta)*sin(theta) + variance_add_y);
    
    d_x = cos(theta)*d_noisy; 
    d_y = sin(theta)*d_noisy; 
    
    sdt_add_sim_x = std(d_x);
    sdt_add_sim_y = std(d_y);
     % write to df
    calculated_additional_noise_x = [calculated_additional_noise_x,std_add_x];
    calculated_additional_noise_y = [calculated_additional_noise_y,std_add_y];
    additional_noise_x = [additional_noise_x,sdt_add_sim_x];
    additional_noise_y = [additional_noise_y,sdt_add_sim_y];
    
     
end

stdev_final = std(diff_final)*std(diff_final);
stdev_final_cov = mean(var_final);
totalvarfinal = mean(var_final)+stdev_final;
ratiocool = sqrt(totalvarfinal)/sqrt(stdev_final_cov);


figure(1);
plot(time_agent_1(1:end),calculated_additional_noise, 'Color', 'green','DisplayName','calculated variance')
hold on;
plot(time_agent_1(1:end) ,additional_noise, 'Color', 'blue','DisplayName','simulated variance')
%plot(time_agent_1 ,(sqrt((xposa_agent_1_ground-xposa_agent_1_est).*(xposa_agent_1_ground-xposa_agent_1_est)+(yposa_agent_1_ground-yposa_agent_1_est).*(yposa_agent_1_ground-yposa_agent_1_est))), 'Color', 'red','DisplayName','error XY')
hold off;
xlabel('time')
ylabel('variance noise')

title('Calculated vs. simulated noise variance') 
lgd = legend;
lgd.NumColumns = 1;
%matlab2tikz('Covariance_validation.tex');

figure(2);
plot(time_agent_1(1:end),calculated_additional_noise_x, 'Color', 'green','DisplayName','calculated noise x direction')
hold on;
plot(time_agent_1(1:end) ,additional_noise_x, 'Color', 'blue','DisplayName','simulated noise x direction')
%plot(time_agent_1 ,(sqrt((xposa_agent_1_ground-xposa_agent_1_est).*(xposa_agent_1_ground-xposa_agent_1_est)+(yposa_agent_1_ground-yposa_agent_1_est).*(yposa_agent_1_ground-yposa_agent_1_est))), 'Color', 'red','DisplayName','error XY')
hold off;
xlabel('time')
ylabel('std noise')

title('Calculated vs. simulated additional noise std x direction') 
lgd = legend;
lgd.NumColumns = 1;


figure(3);
plot(time_agent_1(1:end),calculated_additional_noise_y, 'Color', 'green','DisplayName','calculated noise y direction')
hold on;
plot(time_agent_1(1:end) ,additional_noise_y, 'Color', 'blue','DisplayName','simulated noise y direction')
%plot(time_agent_1 ,(sqrt((xposa_agent_1_ground-xposa_agent_1_est).*(xposa_agent_1_ground-xposa_agent_1_est)+(yposa_agent_1_ground-yposa_agent_1_est).*(yposa_agent_1_ground-yposa_agent_1_est))), 'Color', 'red','DisplayName','error XY')
hold off;
xlabel('time')
ylabel('std noise')

title('Calculated vs. simulated additional noise std y direction') 
lgd = legend;
lgd.NumColumns = 1;



figure(4);
plot(time_agent_1(1:end),econsist, 'Color', 'green','DisplayName','calculated noise y direction')
hold on;
plot(time_agent_1(1:end) ,additional_noise_y, 'Color', 'blue','DisplayName','simulated noise y direction')
%plot(time_agent_1 ,(sqrt((xposa_agent_1_ground-xposa_agent_1_est).*(xposa_agent_1_ground-xposa_agent_1_est)+(yposa_agent_1_ground-yposa_agent_1_est).*(yposa_agent_1_ground-yposa_agent_1_est))), 'Color', 'red','DisplayName','error XY')
hold off;
xlabel('time')
ylabel('EKF Consistency')

title('EKF consistency') 
lgd = legend;
lgd.NumColumns = 1;

