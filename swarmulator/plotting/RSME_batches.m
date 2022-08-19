



%import data for agent 1 where no inter_agent ranging takes place

rmse_list_UWB_D = [];
rmse_list_NO_UWB = [];
rmse_list_UWB_ND = [];
for c = 1:5
  %  if(size(data_agents,1)<=250)
  %  end
    
   % if(size(data_agents,1)>250)
   %     
  
    
    data_agents = importdata(sprintf('batchtest_2022-03-23-13:31:50/log_%d.txt',c)) ;
    time_agent_1 = [];
    agent_agent_1 = [];
    totalposa_agent_1_ground = [];
    xposa_agent_1_ground = [];
    yposa_agent_1_ground = [];
    totalposa_agent_1_est = [];
    xposa_agent_1_est = [];
    yposa_agent_1_est = [];
    totalposa_agent_1_traj = [];
    xposa_agent_1_traj = [];
    yposa_agent_1_traj = [];
    zpos_agent_1 = [];
    for(u=1:size(data_agents,1))
       if(data_agents(u,2)==2)
            time_agent_1= [time_agent_1,data_agents(u,1)];
            agent_agent_1= [agent_agent_1,data_agents(u,2)];
            xposa_agent_1_ground= [xposa_agent_1_ground,data_agents(u,3)];
            yposa_agent_1_ground= [yposa_agent_1_ground,data_agents(u,4)];
            xposa_agent_1_est= [xposa_agent_1_est,data_agents(u,5)];
            yposa_agent_1_est= [yposa_agent_1_est,data_agents(u,6)];
            xposa_agent_1_traj= [xposa_agent_1_traj,data_agents(u,7)];
            yposa_agent_1_traj= [yposa_agent_1_traj,data_agents(u,8)];
            zpos_agent_1= [zpos_agent_1,data_agents(u,5)];
       end
    end
    totalposa_agent_1_est = sqrt((yposa_agent_1_est.*yposa_agent_1_est)+(xposa_agent_1_est.*xposa_agent_1_est));
    totalposa_agent_1_ground = sqrt((yposa_agent_1_ground.*yposa_agent_1_ground)+(xposa_agent_1_ground.*xposa_agent_1_ground)); 
    totalposa_agent_1_traj = sqrt((yposa_agent_1_traj.*yposa_agent_1_traj)+(xposa_agent_1_traj.*xposa_agent_1_traj)); 
    rmse_UWB_ND=sqrt(sum((totalposa_agent_1_est(20:end)-totalposa_agent_1_ground(20:end)).^2)/numel(totalposa_agent_1_ground(20:end)));
    rmse_list_UWB_ND= [rmse_list_UWB_ND,rmse_UWB_ND];
    end

    

for c = 1:5
    data_agents = importdata(sprintf('batchtest_2022-03-23-13:46:35/log_%d.txt',c)) ;
    
   % if(size(data_agents,1)<=250)
   % end
    
   % if(size(data_agents,1)>250)
        
    
    time_agent_1 = [];
agent_agent_1 = [];
xposa_agent_1_ground = [];
yposa_agent_1_ground = [];
totalposa_agent_1_ground = [];
xposa_agent_1_est = [];
totalposa_agent_1_est = [];
yposa_agent_1_est = [];
xposa_agent_1_traj = [];
yposa_agent_1_traj = [];
totalposa_agent_1_traj = [];
zpos_agent_1 = [];
    for(u=1:size(data_agents,1))
       if(data_agents(u,2)==2)
            time_agent_1= [time_agent_1,data_agents(u,1)];
            agent_agent_1= [agent_agent_1,data_agents(u,2)];
            xposa_agent_1_ground= [xposa_agent_1_ground,data_agents(u,3)];
            yposa_agent_1_ground= [yposa_agent_1_ground,data_agents(u,4)];
            xposa_agent_1_est= [xposa_agent_1_est,data_agents(u,5)];
            yposa_agent_1_est= [yposa_agent_1_est,data_agents(u,6)];
            xposa_agent_1_traj= [xposa_agent_1_traj,data_agents(u,7)];
            yposa_agent_1_traj= [yposa_agent_1_traj,data_agents(u,8)];
            zpos_agent_1= [zpos_agent_1,data_agents(u,5)];
       end
    end
    totalposa_agent_1_est = sqrt((yposa_agent_1_est.*yposa_agent_1_est)+(xposa_agent_1_est.*xposa_agent_1_est));
    totalposa_agent_1_ground = sqrt((yposa_agent_1_ground.*yposa_agent_1_ground)+(xposa_agent_1_ground.*xposa_agent_1_ground)); 
    totalposa_agent_1_traj = sqrt((yposa_agent_1_traj.*yposa_agent_1_traj)+(xposa_agent_1_traj.*xposa_agent_1_traj)); 
    rmse_UWB_D=sqrt(sum((totalposa_agent_1_est(20:end)-totalposa_agent_1_ground(20:end)).^2)/numel(totalposa_agent_1_ground(20:end)));
    rmse_list_UWB_D= [rmse_list_UWB_D,rmse_UWB_D];
    end


for c = 1:10
    data_agents = importdata(sprintf('batchtest_3s1d_nodynamicrang_80hzd40hzs/log_%d.txt',c)) ;
    time_agent_1 = [];
agent_agent_1 = [];
xposa_agent_1_ground = [];
yposa_agent_1_ground = [];
xposa_agent_1_est = [];
yposa_agent_1_est = [];
xposa_agent_1_traj = [];
yposa_agent_1_traj = [];
zpos_agent_1 = [];
    for(u=1:size(data_agents,1))
       if(data_agents(u,2)==2)
            time_agent_1= [time_agent_1,data_agents(u,1)];
            agent_agent_1= [agent_agent_1,data_agents(u,2)];
            xposa_agent_1_ground= [xposa_agent_1_ground,data_agents(u,3)];
            yposa_agent_1_ground= [yposa_agent_1_ground,data_agents(u,4)];
            xposa_agent_1_est= [xposa_agent_1_est,data_agents(u,5)];
            yposa_agent_1_est= [yposa_agent_1_est,data_agents(u,6)];
            xposa_agent_1_traj= [xposa_agent_1_traj,data_agents(u,7)];
            yposa_agent_1_traj= [yposa_agent_1_traj,data_agents(u,8)];
            zpos_agent_1= [zpos_agent_1,data_agents(u,5)];
       end
    end
    
    rmse_NO_UWB=sqrt(sum((xposa_agent_1_est(100:end)-xposa_agent_1_ground(100:end)).^2)/numel(xposa_agent_1_ground(100:end)));
    rmse_list_NO_UWB= [rmse_list_NO_UWB,rmse_NO_UWB];
end

A = rmse_list_UWB_ND';
B = rmse_list_UWB_D';

group = [    ones(size(A));
         2 * ones(size(B))];
figure
sgtitle('2 agents Casestudy no exchange vs. covariance rejection') 
boxplot([A; B],group)
ylim([0 0.25])
set(gca,'XTickLabel',{'No exchange a2','Covariance rejection a2'})


figure
boxplot([rmse_list_NO_UWB'])
set(gca,'XTickLabel',{'NO UWB'})
