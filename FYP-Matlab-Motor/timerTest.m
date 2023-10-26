clc
clear all

% Set the desired duration (in seconds)
duration = 2; % Run for 10 seconds
f = 50;

% Initialize the timer
tic;
elapsed_seconds = 0;

while elapsed_seconds < duration
    % Check if the elapsed time is greater than or equal to the desired interval
    if toc >= 1/f
        % Get the current elapsed seconds
        elapsed_seconds = toc + elapsed_seconds;
        
        % Print the elapsed seconds
        fprintf('Elapsed seconds: %f\n', elapsed_seconds);
        
        % Reset the timer
        tic;
    end
    
    % Put your other code here or use a pause to avoid busy-waiting
    % pause(0.01); % Optional: sleep for a short time to reduce CPU usage
end

% Add any cleanup code here
disp('Program stopped.');
