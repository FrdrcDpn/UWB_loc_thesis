% we are simulating with 4 agents 
% importing data for agent 1
% for the case where no inter-agent ranging takes place 
% and for the case where inter-agent ranging does take place
clear all
nees_list_4_quads_CI_CU = [];
rmse_list_4_quads_CI_CU = [];
nees_list_4_quads_CU = [];
rmse_list_4_quads_CU = [];
nees_list_4_quads_vanilla = [];
rmse_list_4_quads_vanilla = [];

for alg=0:4
        if alg == 0
            algo = '4_quads_CI_CU';
            algod = 'quads_CI_CU';
        end
        if alg == 1
            algo = '4_quads_CU';
            algod = 'quads_CU';
        end
        if alg == 2
            algo = '4_quads_vanilla';
            algod = 'quads_vanilla';
        end
        if alg == 3
            algo = '1_quads';
            algod = 'quads';
        end
        nees_list = [];
        rmse_list = [];
    for(u=1:10)
        if isfile(sprintf('/Users/frederic/Desktop/cfsuite_working/crazyflie-suite/data/limited_coverage/%s/logfin_%d.csv',algo,u))
            Tbl = readtable(sprintf('/Users/frederic/Desktop/cfsuite_working/crazyflie-suite/data/limited_coverage/%s/logfin_%d.csv',algo,u));
            %3 quaddos : 0.3458 0.3537 0.3748 0.5534 
            % 0.3609 0.3490 
            % Easiest, Most Direct
            start = 1;
            endvar = 0;
            optitrack_shift =0;
            %import data for agent 1 where inter_agent ranging takes place
            time_agent_1_cyberzoo = Tbl.timeTick;
            time_agent_1_cyberzoo = time_agent_1_cyberzoo(1:end-200);

            agent_agent_1_cyberzoo = [];

            xposa_agent_1_cyberzoo_ground = Tbl.otX0;
            xposa_agent_1_cyberzoo_ground = xposa_agent_1_cyberzoo_ground(1:end-200);

            yposa_agent_1_cyberzoo_ground = Tbl.otY0;
            yposa_agent_1_cyberzoo_ground = yposa_agent_1_cyberzoo_ground(1:end-200);

            xposa_agent_1_cyberzoo_est = Tbl.stateX;
            xposa_agent_1_cyberzoo_est = xposa_agent_1_cyberzoo_est(1:end-200);

            yposa_agent_1_cyberzoo_est = Tbl.stateY;
            yposa_agent_1_cyberzoo_est = yposa_agent_1_cyberzoo_est(1:end-200);

            cyberzoo_varx = Tbl.varx;
            cyberzoo_varx = cyberzoo_varx(1:end-200);

            cyberzoo_vary = Tbl.vary;
            cyberzoo_vary = cyberzoo_vary(1:end-200);

            zpos_agent_1_cyberzoo = [];

            nees_UWB_D_cyb = 0;

            totalposa_agent_1_cyberzoo_est = sqrt((yposa_agent_1_cyberzoo_est.*yposa_agent_1_cyberzoo_est)+(xposa_agent_1_cyberzoo_est.*xposa_agent_1_cyberzoo_est));
            totalposa_agent_1_cyberzoo_ground = sqrt((yposa_agent_1_cyberzoo_ground.*yposa_agent_1_cyberzoo_ground)+(xposa_agent_1_cyberzoo_ground.*xposa_agent_1_cyberzoo_ground));

            rmse_cyb=sqrt(sum((xposa_agent_1_cyberzoo_est(1:end)-xposa_agent_1_cyberzoo_ground(1:end)).^2)/numel(xposa_agent_1_cyberzoo_ground(1:end)));
            rmse_list = [rmse_list,rmse_cyb];

            for ind = 1:length(xposa_agent_1_cyberzoo_ground)
            x = xposa_agent_1_cyberzoo_ground(ind) - xposa_agent_1_cyberzoo_est(ind);
            y = yposa_agent_1_cyberzoo_ground(ind) -yposa_agent_1_cyberzoo_est(ind);
            X = [x;y];
            P= [10*cyberzoo_varx(ind) 0 ; 0 10*cyberzoo_vary(ind)];
            nees_UWB_D_cyb = nees_UWB_D_cyb + (transpose(X)/P)*X;
            end
            nees_UWB_D_cyb = nees_UWB_D_cyb/length(xposa_agent_1_cyberzoo_ground);
            nees_list= [nees_list,nees_UWB_D_cyb];
        end
    end
    data.(algod).rmse = rmse_list;
    data.(algod).rmse_avg = mean(rmse_list);
    data.(algod).nees = nees_list;
    data.(algod).nees_avg = mean(nees_list);
end


for alg=0:4
        if alg == 0
            algo = '4_quads_CI_CU';
            algod = 'quads_CI_CU';
        end
        if alg == 1
            algo = '4_quads_CU';
            algod = 'quads_CU';
        end
        if alg == 2
            algo = '4_quads_vanilla';
            algod = 'quads_vanilla';
        end
        if alg == 3
            algo = '1_quads';
            algod = 'quads';
        end
        nees_list = [];
        rmse_list = [];
    for(u=1:10)
        if isfile(sprintf('/Users/frederic/Desktop/cfsuite_working/crazyflie-suite/data/greenhousetest/%s/logfin_%d.csv',algo,u))
            Tbl = readtable(sprintf('/Users/frederic/Desktop/cfsuite_working/crazyflie-suite/data/greenhousetest/%s/logfin_%d.csv',algo,u));
            %3 quaddos : 0.3458 0.3537 0.3748 0.5534 
            % 0.3609 0.3490 
            % Easiest, Most Direct
            start = 1;
            endvar = 0;
            optitrack_shift =0;
            %import data for agent 1 where inter_agent ranging takes place
            time_agent_1_cyberzoo = Tbl.timeTick;
            time_agent_1_cyberzoo = time_agent_1_cyberzoo(1:end-200);

            agent_agent_1_cyberzoo = [];

            xposa_agent_1_cyberzoo_ground = Tbl.otX0;
            xposa_agent_1_cyberzoo_ground = xposa_agent_1_cyberzoo_ground(1:end-200);

            yposa_agent_1_cyberzoo_ground = Tbl.otY0;
            yposa_agent_1_cyberzoo_ground = yposa_agent_1_cyberzoo_ground(1:end-200);

            xposa_agent_1_cyberzoo_est = Tbl.stateX;
            xposa_agent_1_cyberzoo_est = xposa_agent_1_cyberzoo_est(1:end-200);

            yposa_agent_1_cyberzoo_est = Tbl.stateY;
            yposa_agent_1_cyberzoo_est = yposa_agent_1_cyberzoo_est(1:end-200);

            cyberzoo_varx = Tbl.varx;
            cyberzoo_varx = cyberzoo_varx(1:end-200);

            cyberzoo_vary = Tbl.vary;
            cyberzoo_vary = cyberzoo_vary(1:end-200);

            zpos_agent_1_cyberzoo = [];

            nees_UWB_D_cyb = 0;

            totalposa_agent_1_cyberzoo_est = sqrt((yposa_agent_1_cyberzoo_est.*yposa_agent_1_cyberzoo_est)+(xposa_agent_1_cyberzoo_est.*xposa_agent_1_cyberzoo_est));
            totalposa_agent_1_cyberzoo_ground = sqrt((yposa_agent_1_cyberzoo_ground.*yposa_agent_1_cyberzoo_ground)+(xposa_agent_1_cyberzoo_ground.*xposa_agent_1_cyberzoo_ground));

            rmse_cyb=sqrt(sum((xposa_agent_1_cyberzoo_est(1:end)-xposa_agent_1_cyberzoo_ground(1:end)).^2)/numel(xposa_agent_1_cyberzoo_ground(1:end)));
            rmse_list = [rmse_list,rmse_cyb];
            
            for ind = 1:length(xposa_agent_1_cyberzoo_ground)
            x = xposa_agent_1_cyberzoo_ground(ind) - xposa_agent_1_cyberzoo_est(ind);
            y = yposa_agent_1_cyberzoo_ground(ind) -yposa_agent_1_cyberzoo_est(ind);
            X = [x;y];
            P= [10*cyberzoo_varx(ind) 0 ; 0 10*cyberzoo_vary(ind)];
            nees_UWB_D_cyb = nees_UWB_D_cyb + (transpose(X)/P)*X;
            end
            nees_UWB_D_cyb = nees_UWB_D_cyb/length(xposa_agent_1_cyberzoo_ground);
            nees_list= [nees_list,nees_UWB_D_cyb];
        end
    end
    greenhousedata.(algod).rmse = rmse_list;
    greenhousedata.(algod).rmse_avg = mean(rmse_list);
    greenhousedata.(algod).nees = nees_list;
    greenhousedata.(algod).nees_avg = mean(nees_list);
end


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
nees_list_UWB_D = [];
nees_UWB_D = 0;

c1 = [];
c2 = [];
c3 = [];
c4 = [];
c5 = [];
c6 = [];
noise1 = [];
noise2 = [];
noise3 = [];
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
c_1 = [];
c_2 = [];
c_3 = [];
c_4 = [];


%3 quad no : 0.0909

n_quadrotors =1;
data_agents = importdata('log_2022-07-05-11:09:22.txt') ;
    for(u=1:size(data_agents,1))
       for(agent=1:n_quadrotors)
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
             c_1 = [c_1, data_agents(u,18)];
             c_2 = [c_2, data_agents(u,19)];
             c_3 = [c_3, data_agents(u,20)];
               c_4 = [c_4, data_agents(u,21)];
            noise2= [noise2,data_agents(u,25)];
            noise3= [noise3,data_agents(u,26)];
            if not( isnan(data_agents(u,24)))
              noise1= [noise1,data_agents(u,24)];
            end
       end
       end
    end
    
  
STD = std(noise1);
%% calculating rmse of x location

totalposa_agent_1_est = sqrt((yposa_agent_1_est.*yposa_agent_1_est)+(xposa_agent_1_est.*xposa_agent_1_est));
totalposa_agent_1_ground = sqrt((yposa_agent_1_ground.*yposa_agent_1_ground)+(xposa_agent_1_ground.*xposa_agent_1_ground)); 
totalposa_agent_1_traj = sqrt((yposa_agent_1_traj.*yposa_agent_1_traj)+(xposa_agent_1_traj.*xposa_agent_1_traj)); 
rmse=sqrt(sum((totalposa_agent_1_est(1:end)-totalposa_agent_1_ground(1:end)).^2)/numel(totalposa_agent_1_ground(1:end)));
for ind = 1:length(xposa_agent_1_ground)
x = xposa_agent_1_ground(ind) - xposa_agent_1_est(ind);
y = yposa_agent_1_ground(ind) - yposa_agent_1_est(ind);
X = [x;y];
P= [c_1(ind) c_2(ind); c_4(ind) c_3(ind)];
nees_UWB_D = nees_UWB_D + (transpose(X)/P)*X;
end
nees_UWB_D = nees_UWB_D/length(xposa_agent_1_ground);
nees_list_UWB_D= [nees_list_UWB_D,nees_UWB_D];



figure(2);
h = zeros(1,12);
% h(1) = plot(xposa_agent_1_ground, yposa_agent_1_ground, 'Color',
% 'red','DisplayName','ground truth');
%hold on 
%h(2) =plot(xposa_agent_1_est, yposa_agent_1_est, 'Color',
%'blue','DisplayName','state estimate');
%hold on



h(3) =plot(yposa_agent_1_traj,xposa_agent_1_traj, 'Color', 'red','DisplayName','trajectory');
hold on
%h(4) =plot(-7.5,-2.5,'.','MarkerSize',40, 'Color', 'green', 'Displayname', 'Initial position');
h(4) =plot(-12,-1.5,'.','MarkerSize',40, 'Color', 'blue', 'DisplayName', 'UWB Anchors');
h(5) =plot(12,-1.5,'.','MarkerSize',40, 'Color', 'blue');
h(6) =plot(-12,-3,'.','MarkerSize',40, 'Color', 'blue');
h(7) =plot(12,-3,'.','MarkerSize',40, 'Color', 'blue');
h(8) =plot(-12,1.5,'.','MarkerSize',40, 'Color', 'blue');
h(9) =plot(12,1.5,'.','MarkerSize',40, 'Color', 'blue');
h(10) =plot(-12,3,'.','MarkerSize',40, 'Color', 'blue');
h(11) =plot(12,3,'.','MarkerSize',40, 'Color', 'blue');
rectangle('Position',[-6.5 -3.25 13 6.5],'FaceColor',[0 1 0 0.1],'EdgeColor','g',...
    'LineWidth',3)
h(2) = line(NaN,NaN,'LineWidth',2,'Color','g', 'DisplayName', 'No UWB Coverage');

legend(h(2:4));

%hold off
xlim([-13 13])
ylim([-4 4])

xlabel('X position [m]')
ylabel('Y position [m]')
%title({"Simulated trajectory Crazyflie","rmse=" +rmse})
title({"Simulated test-setup"})
matlab2tikz('trajectory.tex');
%% plotting error in x and y direction for cyberzoo
figure(3);
subplot(4,1,1) 
plot(time_agent_1,xposa_agent_1_ground-xposa_agent_1_est, 'Color', 'red');
title('Simulation error X position in time')
xlabel('time [s]')
ylabel('error in X [m]')

subplot(4,1,2) 

plot(time_agent_1 ,yposa_agent_1_ground-yposa_agent_1_est, 'Color', 'red');
xlabel('time [s]')
ylabel('error in Y [m]')
title('Simulation error Y position in time') 

subplot(4,1,3) 
plot(time_agent_1_cyberzoo/1000,xposa_agent_1_cyberzoo_ground-xposa_agent_1_cyberzoo_est, 'Color', 'red');
title('Cyberzoo error X position in time')
xlabel('time [s]')
ylabel('error in X [m]')


subplot(4,1,4) 
plot(time_agent_1_cyberzoo/1000 ,yposa_agent_1_cyberzoo_ground-yposa_agent_1_cyberzoo_est, 'Color', 'red');
title('Cyberzoo error Y position in time')
xlabel('time [s]')
ylabel('error in Y [m]')


sgtitle('Cyberzoo & Simulation error X/Y position in time') 


%% plotting x and y direction for simulation
figure(4);

subplot(4,1,1)
plot(time_agent_1,xposa_agent_1_ground, 'Color', 'red','DisplayName','ground truth')
hold on;
plot(time_agent_1 ,xposa_agent_1_est, 'Color', 'blue','DisplayName','state estimate')
hold off;
xlabel('time [s]')

ylabel('X position [m]')

title('Simulation X position in time') 
lgd = legend;
lgd.NumColumns = 1;

subplot(4,1,2)

plot(time_agent_1,yposa_agent_1_ground, 'Color', 'red','DisplayName','ground truth')
hold on;
plot(time_agent_1 ,yposa_agent_1_est, 'Color', 'blue','DisplayName','state estimate')
hold off;
xlabel('time [s]')

ylabel('Y position [m]')
title('Simulation Y position in time') 
lgd = legend;
lgd.NumColumns = 1;


subplot(4,1,3) 
plot(time_agent_1_cyberzoo/1000,xposa_agent_1_cyberzoo_ground, 'Color', 'red','DisplayName','ground truth')
hold on;
plot(time_agent_1_cyberzoo/1000 ,xposa_agent_1_cyberzoo_est, 'Color', 'blue','DisplayName','state estimate')
hold off;
xlabel('time [s]')

ylabel('X position [m]')
title('Cyberzoo X position in time') 
lgd = legend;
lgd.NumColumns = 1;


subplot(4,1,4)
plot(time_agent_1_cyberzoo/1000,yposa_agent_1_cyberzoo_ground, 'Color', 'red','DisplayName','ground truth')
hold on;
plot(time_agent_1_cyberzoo/1000 ,yposa_agent_1_cyberzoo_est, 'Color', 'blue','DisplayName','state estimate')
hold off;
xlabel('time [s]')
ylabel('Y position [m]')

title('Cyberzoo Y position in time') 
lgd = legend;
lgd.NumColumns = 1;


sgtitle('Cyberzoo & Simulation state estimate & ground truth Y/Y position in time') 
%
%%figure(5);
%plot(time_agent_1,contr_x, 'Color', 'red','DisplayName',' x')
%%hold on;
%plot(time_agent_1 ,contr_desx, 'Color', 'blue','DisplayName',' x')
%hold off;
%xlabel('time [s]')
%ylabel('Y position [m]')

%title('Cyberzoo Y position in time') 
%lgd = legend;
%lgd.NumColumns = 1;


%figure(6);
%plot(time_agent_1,contr_y, 'Color', 'red','DisplayName',' y')
%hold on;
%plot(time_agent_1 ,contr_desy, 'Color', 'blue','DisplayName',' y')
% hold off;
% xlabel('time [s]')
% ylabel('Y position [m]')
% 
% title('Cyberzoo Y position in time') 
% lgd = legend;
% lgd.NumColumns = 1;
% 
% 
% figure(6);
% plot(time_agent_1,c1, 'Color', 'red','DisplayName','x')
% hold on;
% plot(time_agent_1 ,c3, 'Color', 'blue','DisplayName','y')
% hold on;
% plot(time_agent_1 ,rmse, 'Color', 'blue','DisplayName','error xy')
% hold off;
% xlabel('time')
% ylabel('Phi')
% 
% title('Covariancevxvy') 
% lgd = legend;
% lgd.NumColumns = 1;
% 
% 
% 
 figure(7);
 plot(time_agent_1,sqrt(c1), 'Color', 'green','DisplayName','x')
 hold on;
% plot(time_agent_1 ,c3, 'Color', 'blue','DisplayName','y')
 plot(time_agent_1 ,xposa_agent_1_ground-xposa_agent_1_est,'DisplayName','error XY')
 hold off;
 xlabel('time')
 ylabel('magnitude')
% 
% title('Covariance matrix entry for X and Y') 
 lgd = legend;
 lgd.NumColumns = 1;

%figure(8);
%x = noise1-1;
% Plot the curve alone
%
% I could have retrieved values and edges from h (they are stored in the
% Values and BinEdges properties respectively) but I wanted to show how
% to get this information without actually creating a plot
%histogram(x,100, 'Normalization', 'probability');
 figure(8);
y = [data.quads.rmse_avg data.quads.nees_avg  ; data.quads_vanilla.rmse_avg data.quads_vanilla.nees_avg; data.quads_CU.rmse_avg data.quads_CU.nees_avg; data.quads_CI_CU.rmse_avg data.quads_CI_CU.nees_avg];
bar(y)
set(gca,'xticklabel',{'1 MAV','4 MAV Simple','4 MAV CU','4 MAV CI-CU'})
matlab2tikz('bartest.tex')

rmseses = [data.quads.rmse_avg ; data.quads_vanilla.rmse_avg ; data.quads_CU.rmse_avg ; data.quads_CI_CU.rmse_avg];
neeses = [data.quads.nees_avg ; data.quads_vanilla.nees_avg ; data.quads_CU.nees_avg ; data.quads_CI_CU.nees_avg];
figure
a=[rmseses zeros(4,1)  ];
b=[zeros(4,1)    neeses];
[AX,H1,H2] =plotyy([1:4],a, [1:4],b, 'bar', 'bar');
set(H1,'FaceColor','r') % a
set(H2,'FaceColor','b') % b
hLgnd=legend([H1(1) H2(1)],'RMSE','NEES');
set(gca,'xticklabel',{'1 MAV','4 MAV Simple','4 MAV CU','4 MAV CI-CU'})
matlab2tikz('bartest1.tex')