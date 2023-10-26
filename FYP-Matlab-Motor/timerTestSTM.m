clc
clear all

% Set the desired duration (in seconds)
duration = 10; % Run for 10 seconds
f = 50; % 40Hz seems to not have missed data

% Initialize the timer
elapsed_seconds = 0;
i = 1;

SysIDData.timeSTM = zeros(duration*f,1);
SysIDData.voltage = zeros(duration*f,1);

%% Setup STM32 Serial
if ~isempty(instrfind('Status', 'open'))
    fclose(instrfind);
end
stm_device = serialport('COM4', 115200, 'Timeout', 0.025);

tic;

while elapsed_seconds < duration
    % Check if the elapsed time is greater than or equal to the desired interval
    if toc >= 1/f

        % Get the current elapsed seconds
        elapsed_seconds = toc + elapsed_seconds;
        % Reset the timer
        tic

        %STM32 measure
        writeline(stm_device, "getSysID");
        SysIDData.voltage(i) = str2double(readline(stm_device));
        SysIDData.timeSTM(i) = elapsed_seconds;
        
        i = i + 1;
    end
end

% Add any cleanup code here
disp('Program stopped.');
