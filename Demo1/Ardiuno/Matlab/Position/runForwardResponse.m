%% Run Forward Response by Ian Keeffe and Yoon Seok Lee

% Define motor parameters
K=.17; % DC gain [rad/Vs]
sigma=10; % time constant reciprocal [1/s]
% read motor data
load('stepData.mat')
%% Run a Simulation

%
% open the block diagram so it appears in the documentation when published.
% Make sure the block diagram is closed before running the publish function
%
open_system('forwardResponse')
%
% run the simulation
%
out=sim('forwardResponse');
%% A Plot of the results
%
figure()
sgtitle('Response')

subplot(2,1,1)
plot(out.voltage,'--','linewidth',2)
hold on
plot(data(:,1),(data(:,2)),'linewidth',2)
legend('Simulated','Experimental','location','southeast')
hold off
xlabel('Time (s)')
ylabel('Voltage (V)')
title('Time Series Plot: Voltage')

subplot(2,1,2)
plot(out.velocity,'--','linewidth',2)
hold on
plot(data(:,1),data(:,3),'linewidth',2)
hold off
legend('Simulated','Experimental','location','southeast')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('Time Series Plot: Velocity')

