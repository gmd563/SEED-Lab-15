%% Animation of the Position and Orientation of a Vehicle by Ian Keeffe and Yoon Seok Lee

% With SEED_output_position loaded on ardunio run ReadFromArdiuno (from 
% canvas) on matlab to fill the data array

% fills arrays for plotting with ardunio data
time = data(:, 1);
xposition = data(:, 2);
yposition = data(:, 3);
phi = data(:, 4);

% Define shape vertices (vehicle representation)
r_width = 0.5;
r_length = 1;
V = [-r_length/2, -r_length/2, 0, r_length/2, 0;
     -r_width/2,  r_width/2, r_width/2, 0, -r_width/2];

% Create figure and axis
figure;
hold on;
axis([-10 10 -10 10]);
axis manual;  % Prevent axis rescaling during animation
xlabel('X Distance (m)')
ylabel('Y Distance (m)')

% Create a patch object for the vehicle
hPatch = fill(V(1, :), V(2, :), 'y');

for i = 1:length(time)
    % Define rotation matrix
    T = [cos(phi(i)), -sin(phi(i));
         sin(phi(i)),  cos(phi(i))];
    
    % Define center position
    pos = [xposition(i); yposition(i)];
    
    % Find current vertices positions
    v_c = T * V + pos * ones(1, 5);
    
    % Update patch vertices
    set(hPatch, 'XData', v_c(1, :), 'YData', v_c(2, :));
    
    % Update plot
    pause(0.01);  % Minimal pause to force a refresh
end


%% Interpretation
% You can see the robot move through the x y plane. This appears to be very
% accurate and should help with out implementation of path tracing.
