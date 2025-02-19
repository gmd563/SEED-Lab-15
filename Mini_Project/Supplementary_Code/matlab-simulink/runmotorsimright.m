%% Define motor parameters
K=1.3; % DC gain [rad/Vs]
sigma=12; % time constant reciprocal [1/s]
%% read motor data
load('stepData.mat')
%% Run a Simulation

%
% open the block diagram so it appears in the documentation when published.
% Make sure the block diagram is closed before running the publish function
%
open_system('motorsim')
%
% run the simulation
%
out=sim('motorsim');
%% A Plot of the results
%
figure
subplot(2,1,1)
plot(out.voltage,'--','linewidth',2)
hold on
plot(data(:,1),data(:,2),'linewidth',2)
legend('Simulated','Experimental','location','southeast')
hold off
xlabel('Time (s)')
ylabel('Voltage (V)')
subplot(2,1,2)
plot(out.velocity,'--','linewidth',2)
hold on
plot(data(:,1),data(:,3),'linewidth',2)
hold off
legend('Simulated','Experimental','location','southeast')
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')


