%% Run Motor Control by Ian Keeffe and Yoon Seok Lee

% We used simulink to make a appropriate model of a porportional
% controller for a dc motor. This is motor_control
% K and Sigma were found in runmotorsim left and runmotorsimright.
% With SEED_output_velocity loaded on ardunio run ReadFromArdiuno (from 
% canvas) on matlab to fill the data array

%% Define motor parameters
% K=.33; % DC gain [rad/Vs]
% sigma=5; % time constant reciprocal [1/s]
K=.65; % DC gain [rad/Vs]
sigma=11; % time constant reciprocal [1/s]
kp = 6.5;
%% Run a Simulation

%
% open the block diagram so it appears in the documentation when published.
% Make sure the block diagram is closed before running the publish function
%
open_system('controlledForwardResponse')
%
% run the simulation
%
out=sim('controlledForwardResponse');
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
plot(out.velocity,'linewidth',2)
hold on
plot(out.desiredVelocity,'--','linewidth',2)
plot(data(:,1),data(:,3),'linewidth',2)
hold off
legend('Simulated','Desired','Experimental','location','southeast')
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')

