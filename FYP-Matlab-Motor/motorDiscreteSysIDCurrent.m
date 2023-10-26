clc
clear all

%% SYS_ID_DATA_LOAD.mat

%% Load motor data
motorData = {'SYS_ID_DATA_LOAD7.mat', ...
             'SYS_ID_DATA_LOAD8.mat', ...
             'SYS_ID_DATA_LOAD9.mat', ...
             'SYS_ID_DATA_LOAD10.mat', ...
             'SYS_ID_DATA_LOAD11.mat', ...
             'SYS_ID_DATA_LOAD12.mat'};
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
    [timeTemp,currentTemp,velocityTemp,VinTemp,torqueInterpTemp] = motorDataUnpacker(motorData{i});

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

%% Least Squares Method - Torque
[phi,Kw_T] = motorTorqueLSME(velocity, Vin, torqueInterp)

N = 255;
VinTorque = phi*(torqueInterp/N) + Kw_T*velocity;

%% Least Squares Method - Current
[Ra, Kw_I] = motorCurrentLSME(velocity, Vin, current)

VinCurrent = Ra*current + Kw_I*velocity;

%% Root Mean Squared Error
rmse_VinTorque  = sqrt(mean((Vin - VinTorque).^2))
rmse_VinCurrent = sqrt(mean((Vin - VinCurrent).^2))

%% Plot Calculations
fig3 = figure;
set(fig3,'Position', [100,100,800,400])
hold on
grid on

plot(time,VinTorque,'*','color','red')
plot(time,VinCurrent,'*','color',"#EDB120")
plot(time,Vin,'color','blue','linewidth',2)
% plot(time,Vin + rmse_VinTorque,'-.','color','red')
% plot(time,Vin - rmse_VinTorque,'-.','color','red')
% plot(time,Vin + rmse_VinCurrent,'-.','color',"#EDB120")
% plot(time,Vin - rmse_VinCurrent,'-.','color',"#EDB120")
legend('Torque Calculation', 'Current Calculation','Vin')
title('Current System Identification')

saveas(fig3,'currentSysId.png')

%% Validation
% Load motor data
motorData_Validation = 'SYS_ID_DATA_LOAD9.mat';

[rmse_Validation_VinTorque,rmse_Validation_VinCurrent] = motorValidation(motorData_Validation, phi, Kw_T, Ra, Kw_I);




