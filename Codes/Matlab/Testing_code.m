clear;
t_span=[0 10];

% Initial state space vector[q1 q2 q1dot q2dot neg_int_q1 neg_ing_q2]
y0=[0.2, 0.15, 0, 0, 0, 0]; 
% Desired angles for joint 1 and 2
q1_des=0;  
q2_des=0; 

[t,s] = ode45(@(t,state) func(t,state,q1_des,q2_des),t_span,y0);

q1=s(:,1);
q2=s(:,2);
e1 = q1_des - q1;
e2 = q2_des - q2;

% Plotting
figure;
subplot(2,1,1);
plot(t,q1,'LineWidth',1.5);
title('Joint Angle q1 vs Time');
xlabel('Time (s)');
ylabel('q1 (rad)');

subplot(2,1,2);
plot(t,q2,'LineWidth',1.5);
title('Joint Angle q2 vs Time');
xlabel('Time (s)');
ylabel('q2 (rad)');

figure;
subplot(2,1,1);
plot(t,e1,'LineWidth',1.5);
title('error in q1 vs Time');
xlabel('Time (s)');
ylabel('q1 (rad)');

subplot(2,1,2);
plot(t,e2,'LineWidth',1.5);
title('error in q2 vs Time');
xlabel('Time (s)');
ylabel('q2 (rad)');