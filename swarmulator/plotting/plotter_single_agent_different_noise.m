data = importdata('log_2021-09-09-12:23:35.txt') ;
    time = data(:,1);
    agent = data(:,2);
    xposa = data(:,3);
    yposa = data(:,4);
    zpos = data(:,5);
   
 data = importdata('log_2021-09-09-12:24:16.txt') ;
    time = data(:,1);
    agent = data(:,2);
    xposb = data(:,3);
    yposb = data(:,4);
    zpos = data(:,5);
  data = importdata('log_2021-09-09-12:24:52.txt') ;
    time = data(:,1);
    agent = data(:,2);
    xposc = data(:,3);
    yposc = data(:,4);
    zpos = data(:,5);

      data = importdata('log_2021-09-09-12:25:37.txt') ;
    time = data(:,1);
    agent = data(:,2);
    xposd = data(:,3);
    yposd = data(:,4);
    zpos = data(:,5);
    
 
subplot(2,2,1) 
plot(xposa, yposa, 'Color', 'red');
hold on 
plot(20,20,'.','MarkerSize',50, 'Color', 'red')
plot(-20,20,'.','MarkerSize',50, 'Color', 'red')
plot(-20,-20,'.','MarkerSize',50, 'Color', 'red')
hold off
xlim([-20 20])
ylim([-20 20])
rectangle('Position',[-17.5 -17.5 35 35])
title('No noise') 
subplot(2,2,2) 
plot(xposb, yposb, 'Color', 'red');
hold on 
plot(20,20,'.','MarkerSize',50, 'Color', 'red')
plot(-20,20,'.','MarkerSize',50, 'Color', 'red')
plot(-20,-20,'.','MarkerSize',50, 'Color', 'red')
hold off
xlim([-20 20])
ylim([-20 20])
rectangle('Position',[-17.5 -17.5 35 35])
title('Gaussian noise') 
subplot(2,2,3) 


plot(xposc, yposc, 'Color', 'red');
hold on 
plot(20,20,'.','MarkerSize',50, 'Color', 'red')
plot(-20,20,'.','MarkerSize',50, 'Color', 'red')
plot(-20,-20,'.','MarkerSize',50, 'Color', 'red')
hold off
xlim([-20 20])
ylim([-20 20])
rectangle('Position',[-17.5 -17.5 35 35])
title('ht cauchy noise') 
subplot(2,2,4) 

plot(xposd, yposd, 'Color', 'red');
hold on 
plot(20,20,'.','MarkerSize',50, 'Color', 'red')
plot(-20,20,'.','MarkerSize',50, 'Color', 'red')
plot(-20,-20,'.','MarkerSize',50, 'Color', 'red')
hold off
xlim([-20 20])
ylim([-20 20])
rectangle('Position',[-17.5 -17.5 35 35])
title('ht gamma noise') 
sgtitle('Simulation state xy 300s') 

