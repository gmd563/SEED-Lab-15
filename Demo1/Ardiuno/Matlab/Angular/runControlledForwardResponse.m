%% Run Motor Control by Ian Keeffe and Yoon Seok Lee

% We used simulink to make a appropriate model of a porportional
% controller for a dc motor. This is motor_control
% K and Sigma were found in runmotorsim left and runmotorsimright.
% With SEED_output_velocity loaded on ardunio run ReadFromArdiuno (from 
% canvas) on matlab to fill the data array

%% Define motor parameters
K=.65; % DC gain [rad/Vs]
sigma=11; % time constant reciprocal [1/s]
kp = 6.5;
%% Run a Simulation

%
% open the block diagram so it appears in the documentation when published.
% Make sure the block diagram is closed before running the publish function
%
open_system('ControlledAngleResponse')
%
% run the simulation
%
out=sim('ControlledAngleResponse.slx');
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
plot(data(:,1),data(:,4),'linewidth',2)
hold off
legend('Simulated','Desired','Experimental','location','southeast')
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')


% This is a plot of our desired velocity vs actual velocity. 
% by changing kp we can make the porportional control act as
% we desire. We were looking for a response above 80% of the desired
% velocity and no oscilations from noise.

%% Interpretation
% The first obvious thing that can be seen is the significant max in
% voltage during the motor startup. This is from the ardunio and shown this
% way because the arduino graph does not account for the saturation from
% the max voltage only being around 7.5. 
%
% Also there is a significant amount more noise in the voltage and 
% velocity. We believe this is because of noise in the encoder. This causes
% more noise in both the positon and voltage because of the feedback
% control.
