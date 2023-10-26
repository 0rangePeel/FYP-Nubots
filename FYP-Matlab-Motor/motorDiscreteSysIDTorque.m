clc
clear all

%% SYS_ID_DATA_LOAD.mat

%% Load motor data
motorData = {'SYS_ID_DATA_TORQUE1.mat', ...
             'SYS_ID_DATA_TORQUE2.mat', ...
             'SYS_ID_DATA_TORQUE3.mat', ...
             'SYS_ID_DATA_TORQUE4.mat', ...
             'SYS_ID_DATA_TORQUE5.mat', ...
             'SYS_ID_DATA_TORQUE6.mat', ...
             'SYS_ID_DATA_TORQUE7.mat', ...
             'SYS_ID_DATA_TORQUE8.mat'};
% motorData = {'SYS_ID_DATA_LOAD13.mat', ...
%              'SYS_ID_DATA_LOAD14.mat', ...
%              'SYS_ID_DATA_LOAD15.mat', ...
%              'SYS_ID_DATA_LOAD16.mat', ...
%              'SYS_ID_DATA_LOAD17.mat', ...
%              'SYS_ID_DATA_LOAD18.mat'};

time            = [0];
current         = [];
velocity        = [];
Vin             = [];
torqueInterp    = [];

for i = 1:length(motorData)
    [timeTemp,currentTemp,velocityTemp,VinTemp,torqueInterpTemp] = motorDataUnpacker2(motorData{i});

    time = [time; timeTemp + time(end)];
    current = [current; currentTemp];
    velocity = [velocity; velocityTemp];
    Vin = [Vin; VinTemp];
    torqueInterp = [torqueInterp; torqueInterpTemp];

end

time = time(2:end);

%% Data Plotting
figure(1);
plot(time,torqueInterp);
hold on
plot(time,current,'*')
legend('Torque','Current')

figure(2)
plot(time,velocity,'*');
hold on
plot(time,Vin,'*')
legend('Velocity','Vin')

% Gear ratio
N = 255;

% Least Squares estimate value for Ki
num = [current*N];
[num2] = lsqr(num,torqueInterp);
KI = num2(1)

% Plot 1Hz
fig3 = figure;
set(fig3,'Position', [100,100,800,400])
hold on
grid on

plot(time,N*KI*current,'*','color','red')
plot(time,torqueInterp,'color','blue','linewidth',2)
title('Torque System Identification')
xlabel('time (s)')
ylabel('Torque (Nm)')
legend('Calculation Current','Torque')

saveas(fig3,'torqueSysId.png')